#!/usr/bin/env python3
"""BENCH-ONLY FCU stand-in so the real ibvs_controller reaches ALIGN and emits
body-rate commands from real detector input, WITHOUT a flying FCU.

It publishes:
  * mavros/state  -- armed + GUIDED_NOGPS (so the controller's state machine
                     leaves WAIT_ARM and services the target),
  * mavros/local_position/odom -- a level, stationary pose at a fixed altitude
                     (the controller's attitude cascade reads attitude/velocity
                     from here; indoors the real EKF publishes nothing),
and offers dummy set_mode / cmd/arming services so request_engage / land-disarm
do not error.

!!! NEVER run this while the vehicle can fly. It fakes "armed + GUIDED_NOGPS"
and a fake position estimate; it is a desk/bench confirmation aid only. Do NOT
run it alongside real mavros on an armed vehicle. !!!
"""
import os
import rospy
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import (SetMode, SetModeResponse,
                             CommandBool, CommandBoolResponse)
from nav_msgs.msg import Odometry


def main():
    rospy.init_node('bench_fcu_stub')
    alt = float(rospy.get_param('~altitude', 1.5))
    # publish_state: fake armed+GUIDED_NOGPS (pure bench, NO real mavros).
    # publish_odom : fake a level odom so the controller's attitude cascade
    #                has feedback (indoors there is no GPS -> real EKF gives
    #                no local_position/odom).
    # For a test with the REAL FCU armed by the pilot, set publish_state:=false
    # (let mavros own the state) and keep publish_odom:=true.
    publish_state = bool(rospy.get_param('~publish_state', True))
    publish_odom = bool(rospy.get_param('~publish_odom', True))
    fake_services = bool(rospy.get_param('~fake_services', publish_state))
    rospy.logwarn("bench_fcu_stub: BENCH-ONLY. publish_state=%s publish_odom=%s "
                  "(level @ %.2fm). NEVER run near a flyable vehicle.",
                  publish_state, publish_odom, alt)

    state_pub = rospy.Publisher('mavros/state', State, queue_size=1) if publish_state else None
    odom_pub = rospy.Publisher('mavros/local_position/odom', Odometry, queue_size=1) if publish_odom else None

    if fake_services:
        def set_mode_cb(_req):
            return SetModeResponse(mode_sent=True)

        def arming_cb(req):
            rospy.loginfo("bench_fcu_stub: arming service called value=%s", req.value)
            return CommandBoolResponse(success=True, result=0)

        rospy.Service('mavros/set_mode', SetMode, set_mode_cb)
        rospy.Service('mavros/cmd/arming', CommandBool, arming_cb)

    def tick(_ev):
        if state_pub is not None:
            s = State()
            s.header.stamp = rospy.Time.now()
            s.connected = True
            s.armed = True
            s.guided = True
            s.mode = 'GUIDED_NOGPS'
            state_pub.publish(s)

        if odom_pub is not None:
            o = Odometry()
            o.header.stamp = rospy.Time.now()
            o.pose.pose.position.z = alt
            o.pose.pose.orientation.w = 1.0     # level
            odom_pub.publish(o)

    rospy.Timer(rospy.Duration(0.05), tick)   # 20 Hz
    rospy.spin()


if __name__ == '__main__':
    main()
