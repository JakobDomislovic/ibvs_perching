#!/usr/bin/env python

"""
Mock vision module for the ibvs_perching demo (no camera needed).

Speaks the same VISION MODULE INTERFACE as aruco_detector.py: publishes
`ibvs/target_point` (geometry_msgs/PointStamped) with the PIXEL coordinates
a down-facing camera WOULD see for a target at a fixed world position,
computed from real mavros odometry plus an assumed ~horizontal_fov -- the
one thing a real detector would NOT need, since this one has no camera or
image at all and has to fake a plausible projection from scratch:

    point.x  pixel column (u), 0 .. image_width
    point.y  pixel row (v), 0 .. image_height
    point.z  unused, always 0.0 (no depth -- see ibvs_controller.py)

Down camera with image right = body forward: a target at body FLU
(bx, by, bz) sits at optical (bx, -by, -bz). depth = -bz is used ONLY
internally, to project the target into the image and decide whether it is
"in view" -- exactly the privileged ground-truth information a real
detector does not have, which is why this stays a MOCK and never appears
on the published topic. Only publishes while the target is actually below
the vehicle (in "view"), so the TAG_IN_SIGHT logic behaves like with a
real detector.
"""

import math

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped


def clamp(value, low, high):
    return max(low, min(high, value))


class MockArTagPublisher:

    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 15.0)
        self.odom_topic = rospy.get_param('~odom_topic', 'mavros/local_position/odom')
        self.tag_world_position = rospy.get_param('~tag_world_position', [0.0, 0.0, 0.02])
        self.min_depth = rospy.get_param('~min_depth', 0.1)
        # must match the controller's ~image_width/~image_height (set once,
        # together, in the launch file)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        # assumed camera FOV, used only to fake a believable pixel position
        # (default: the kopterworx down-facing camera, 80 deg horizontal)
        self.horizontal_fov = rospy.get_param('~horizontal_fov', 1.3962634)

        self.latest_odom = None

        self.point_pub = rospy.Publisher('ibvs/target_point', PointStamped, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        rospy.loginfo(
            "mock_ar_tag_publisher: faking target at world position %s, "
            "reading odometry from '%s'", self.tag_world_position, self.odom_topic)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.latest_odom is not None:
                self.publish_target_point(self.latest_odom)
            rate.sleep()

    def publish_target_point(self, odom):
        uav_pos = odom.pose.pose.position
        q = odom.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = self.tag_world_position[0] - uav_pos.x
        dy = self.tag_world_position[1] - uav_pos.y
        dz = self.tag_world_position[2] - uav_pos.z

        # world offset -> body FLU (yaw only; a mock does not need full
        # attitude compensation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        body_z = dz

        depth = -body_z          # optical axis points down; used only here,
                                  # never published (see module docstring)
        if depth < self.min_depth:
            return               # target not below the vehicle -> not "in view"

        # tan(angle) / tan(half_fov) -> [-1, 1] fraction of the half-frame,
        # matching exactly how the controller decodes pixels back to a
        # fraction (per-axis, using image_width/image_height -- see
        # ibvs_controller.py's target_callback)
        tan_half_fov = math.tan(self.horizontal_fov / 2.0)
        norm_x = clamp((body_x / depth) / tan_half_fov, -1.0, 1.0)
        norm_y = clamp((-body_y / depth) / tan_half_fov, -1.0, 1.0)

        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera'
        msg.point.x = float(round(self.image_width / 2.0 * (1.0 + norm_x)))
        msg.point.y = float(round(self.image_height / 2.0 * (1.0 + norm_y)))
        msg.point.z = 0.0
        self.point_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('mock_ar_tag_publisher')
    try:
        MockArTagPublisher().run()
    except rospy.ROSInterruptException:
        pass
