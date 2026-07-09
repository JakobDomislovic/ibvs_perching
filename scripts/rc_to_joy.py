#!/usr/bin/env python

"""
RC transmitter -> sensor_msgs/Joy bridge.

Python port of uav_ros_general's rc_to_joy_node, copied into ibvs_perching
so the real-world startup does not depend on the uav_ros_stack: mavros
publishes the raw RC channels on `mavros/rc/in` and this node republishes
them as a `joy` message with the LARICS axis/button layout.

Axis layout (identical to uav_ros_general):
    axes[0] yaw      axes[1] throttle   axes[2] roll   axes[3] pitch
    axes[4] mode switch                 axes[7] sequence switch
    buttons[0] rc_on switch             buttons[5] inspection switch

Channel values in [1100, 1900] us map linearly to [-1, 1]; roll, pitch and
yaw are inverted like the original. Sticks are median-filtered over 5
samples (switches over 15/5) and the stick axes get a deadzone that is
rescaled so the output still reaches +-1 at full deflection.

Parameters (all required, same names as uav_ros_general):
    ~channel/{throttle,roll,pitch,yaw,rc_on,mode}   RC channel indices
    ~joy/deadzone/{x,y,z,yaw}                       stick deadzones
"""

from collections import deque

import rospy
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import Joy

RC_VALUE_MIN = 1100.0
RC_VALUE_MAX = 1900.0
RC_VALUE_MID = (RC_VALUE_MAX + RC_VALUE_MIN) / 2.0

# fixed channels of the two auxiliary switches (as in rc_to_joy_node.cpp)
RC_CHANNEL_INSPECTION = 4
RC_CHANNEL_SEQUENCE = 6

JOY_AXIS_YAW = 0
JOY_AXIS_THROTTLE = 1
JOY_AXIS_ROLL = 2
JOY_AXIS_PITCH = 3
JOY_AXIS_MODE = 4
JOY_AXIS_SEQUENCE = 7
JOY_BUTTON_RC_ON = 0
JOY_BUTTON_INSPECTION = 5


def rc_to_axis(rc_value):
    """[1100, 1900] us -> [-1, 1], clamped."""
    value = (rc_value - RC_VALUE_MID) / ((RC_VALUE_MAX - RC_VALUE_MIN) / 2.0)
    return max(-1.0, min(1.0, value))


def rc_to_button(rc_value):
    return 1 if rc_value > RC_VALUE_MID + 100.0 else 0


def deadzone(value, width):
    """Zero inside the deadzone, rescaled to still reach +-1 outside it."""
    if abs(value) <= width:
        return 0.0
    shifted = value - width if value > 0.0 else value + width
    return shifted / (1.0 - width)


class MedianFilter:

    def __init__(self, size):
        self.samples = deque(maxlen=size)

    def update(self, sample):
        self.samples.append(sample)
        ordered = sorted(self.samples)
        return ordered[len(ordered) // 2]


class RcToJoy:

    def __init__(self):
        self.ch_throttle = rospy.get_param('~channel/throttle')
        self.ch_roll = rospy.get_param('~channel/roll')
        self.ch_pitch = rospy.get_param('~channel/pitch')
        self.ch_yaw = rospy.get_param('~channel/yaw')
        self.ch_rc_on = rospy.get_param('~channel/rc_on')
        self.ch_mode = rospy.get_param('~channel/mode')

        self.dz_x = rospy.get_param('~joy/deadzone/x')
        self.dz_y = rospy.get_param('~joy/deadzone/y')
        self.dz_z = rospy.get_param('~joy/deadzone/z')
        self.dz_yaw = rospy.get_param('~joy/deadzone/yaw')
        rospy.loginfo("rc_to_joy: deadzones x %.2f y %.2f z %.2f yaw %.2f",
                      self.dz_x, self.dz_y, self.dz_z, self.dz_yaw)

        self.filt_throttle = MedianFilter(5)
        self.filt_roll = MedianFilter(5)
        self.filt_pitch = MedianFilter(5)
        self.filt_yaw = MedianFilter(5)
        self.filt_mode = MedianFilter(15)
        self.filt_inspection = MedianFilter(15)

        self.joy_pub = rospy.Publisher('joy', Joy, queue_size=1)
        rospy.Subscriber('mavros/rc/in', RCIn, self.rc_callback, queue_size=1)

    def rc_callback(self, msg):
        if not msg.channels:
            return
        needed = max(self.ch_throttle, self.ch_roll, self.ch_pitch,
                     self.ch_yaw, self.ch_rc_on, self.ch_mode,
                     RC_CHANNEL_INSPECTION, RC_CHANNEL_SEQUENCE)
        if len(msg.channels) <= needed:
            rospy.logwarn_throttle(
                5.0, "rc_to_joy: only %d RC channels, need index %d",
                len(msg.channels), needed)
            return

        joy = Joy()
        joy.header = msg.header
        joy.header.stamp = rospy.Time.now()
        joy.axes = [0.0] * 12
        joy.buttons = [0] * 12

        throttle = rc_to_axis(self.filt_throttle.update(msg.channels[self.ch_throttle]))
        roll = rc_to_axis(self.filt_roll.update(msg.channels[self.ch_roll]))
        pitch = rc_to_axis(self.filt_pitch.update(msg.channels[self.ch_pitch]))
        yaw = rc_to_axis(self.filt_yaw.update(msg.channels[self.ch_yaw]))

        joy.axes[JOY_AXIS_THROTTLE] = deadzone(throttle, self.dz_z)
        joy.axes[JOY_AXIS_ROLL] = deadzone(-roll, self.dz_y)
        joy.axes[JOY_AXIS_PITCH] = deadzone(-pitch, self.dz_x)
        joy.axes[JOY_AXIS_YAW] = -deadzone(yaw, self.dz_yaw)
        joy.axes[JOY_AXIS_MODE] = rc_to_axis(
            self.filt_mode.update(msg.channels[self.ch_mode]))
        joy.axes[JOY_AXIS_SEQUENCE] = rc_to_axis(msg.channels[RC_CHANNEL_SEQUENCE])

        joy.buttons[JOY_BUTTON_RC_ON] = rc_to_button(msg.channels[self.ch_rc_on])
        joy.buttons[JOY_BUTTON_INSPECTION] = rc_to_button(
            self.filt_inspection.update(msg.channels[RC_CHANNEL_INSPECTION]))

        self.joy_pub.publish(joy)


if __name__ == '__main__':
    rospy.init_node('rc_to_joy')
    try:
        RcToJoy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
