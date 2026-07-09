#!/usr/bin/env python

"""
Mock vision module for the ibvs_perching demo (no camera needed).

Speaks the same VISION MODULE INTERFACE as aruco_detector.py: publishes
`ibvs/target_point` (geometry_msgs/PointStamped) with the normalized image
coordinates a down-facing camera WOULD see for a target at a fixed world
position, computed from real mavros odometry:

    point.x  (u - cx)/fx equivalent, positive right in the image
    point.y  (v - cy)/fy equivalent, positive down in the image
    point.z  distance along the optical axis [m]

Down camera with image right = body forward: a target at body FLU
(bx, by, bz) sits at optical (bx, -by, -bz), so
    point.x = bx / -bz,  point.y = -by / -bz,  point.z = -bz
Only publishes while the target is actually below the vehicle (in "view"),
so the TAG_IN_SIGHT logic behaves like with a real detector.
"""

import math

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped


class MockArTagPublisher:

    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 15.0)
        self.odom_topic = rospy.get_param('~odom_topic', 'mavros/local_position/odom')
        self.tag_world_position = rospy.get_param('~tag_world_position', [0.0, 0.0, 0.02])
        self.min_depth = rospy.get_param('~min_depth', 0.1)

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

        depth = -body_z          # optical axis points down
        if depth < self.min_depth:
            return               # target not below the vehicle -> not "in view"

        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera'
        msg.point.x = body_x / depth
        msg.point.y = -body_y / depth
        msg.point.z = depth
        self.point_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('mock_ar_tag_publisher')
    try:
        MockArTagPublisher().run()
    except rospy.ROSInterruptException:
        pass
