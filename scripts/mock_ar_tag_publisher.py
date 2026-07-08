#!/usr/bin/env python

"""
Fakes an AR-tag detector for the ibvs_perching demo.

Instead of a hard-coded constant, this node computes the tag's position
relative to the UAV body (FLU: x-forward, y-left, z-up) from the real
mavros odometry and a fixed tag position in the local ENU frame. This
lets the IBVS controller actually converge in simulation, since the
"detection" reacts to the vehicle really moving in Gazebo.

With the default tag_world_position and the kopterworx default spawn
pose (0, 0, 0.5), the first published detection is (-2, 0, 1) -- exactly
the example from the task: tag detected at (-2, 0, 1), target (0, 0, 1).
"""

import math

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class MockArTagPublisher:

    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)
        self.odom_topic = rospy.get_param('~odom_topic', 'mavros/local_position/odom')
        self.tag_world_position = rospy.get_param('~tag_world_position', [-2.0, 0.0, 1.5])
        self.frame_id = rospy.get_param('~frame_id', 'base_link')

        self.latest_odom = None

        self.tag_pub = rospy.Publisher('ibvs/tag_pose', PoseStamped, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        rospy.loginfo(
            "mock_ar_tag_publisher: faking tag at world position %s, "
            "reading odometry from '%s'", self.tag_world_position, self.odom_topic)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.latest_odom is not None:
                self.publish_relative_tag_pose(self.latest_odom)
            rate.sleep()

    def publish_relative_tag_pose(self, odom):
        uav_pos = odom.pose.pose.position
        q = odom.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = self.tag_world_position[0] - uav_pos.x
        dy = self.tag_world_position[1] - uav_pos.y
        dz = self.tag_world_position[2] - uav_pos.z

        # Rotate the world-frame offset into the body FLU frame (yaw only,
        # a mock detector does not need full attitude compensation).
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        body_z = dz

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = body_x
        msg.pose.position.y = body_y
        msg.pose.position.z = body_z
        msg.pose.orientation.w = 1.0
        self.tag_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('mock_ar_tag_publisher')
    try:
        MockArTagPublisher().run()
    except rospy.ROSInterruptException:
        pass
