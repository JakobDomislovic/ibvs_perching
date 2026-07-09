#!/usr/bin/env python

"""
ArUco vision module for the IBVS controller.

This node is ONE possible vision module: it detects an ArUco marker and
publishes the point the controller should center in the camera image.
Replace it with any other detector that speaks the same interface and the
controller works unchanged.

VISION MODULE INTERFACE (topic `ibvs/target_point`, geometry_msgs/PointStamped):
    point.x  normalized horizontal offset from the image center,
             (u - cx) / fx, positive RIGHT in the image
    point.y  normalized vertical offset from the image center,
             (v - cy) / fy, positive DOWN in the image
    point.z  distance to the target along the optical axis [m],
             or 0.0 if unknown (the controller then only centers X-Y
             and holds altitude)

    Publish ONLY while the target is actually detected -- the controller
    treats fresh messages as "target in sight" (TAG_IN_SIGHT state).

Here the marker's pixel center gives point.x/point.y directly (exact even
if marker_length is miscalibrated), and the pose estimate from the known
marker size provides the optional depth hint in point.z.
"""

import numpy as np

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image

import cv2
import cv2.aruco as aruco


class ArucoDetector:

    def __init__(self):
        self.marker_id = rospy.get_param('~marker_id', 0)
        self.marker_length = rospy.get_param('~marker_length', 0.20)
        # detection rate: frames arriving faster than this are skipped
        # (camera runs at 30 fps, detection at 15 Hz is plenty)
        self.process_rate = rospy.get_param('~process_rate', 15.0)
        dictionary_name = rospy.get_param('~dictionary', 'DICT_4X4_50')
        self.dictionary = aruco.Dictionary_get(getattr(aruco, dictionary_name))
        self.detector_params = aruco.DetectorParameters_create()
        self.last_processed = rospy.Time(0)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()

        self.point_pub = rospy.Publisher('ibvs/target_point', PointStamped, queue_size=1)
        self.debug_pub = rospy.Publisher('ibvs/debug_image', Image, queue_size=1)

        rospy.Subscriber('camera/color/camera_info', CameraInfo,
                         self.camera_info_callback, queue_size=1)
        rospy.Subscriber('camera/color/image_raw', Image,
                         self.image_callback, queue_size=1, buff_size=2 ** 22)

        rospy.loginfo(
            "aruco_detector: vision module for %s id %d (%.2f m), %g Hz",
            dictionary_name, self.marker_id, self.marker_length, self.process_rate)

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D, dtype=np.float64)
            rospy.loginfo("aruco_detector: camera intrinsics received (fx=%.1f)",
                          self.camera_matrix[0, 0])

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        # throttle to process_rate
        now = rospy.Time.now()
        if (now - self.last_processed).to_sec() < 1.0 / self.process_rate:
            return
        self.last_processed = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)

        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                if marker_id != self.marker_id:
                    continue
                self.publish_point(msg.header, marker_corners)
                break

        if self.debug_pub.get_num_connections() > 0:
            debug = frame.copy()
            if ids is not None:
                aruco.drawDetectedMarkers(debug, corners, ids)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def publish_point(self, header, marker_corners):
        # normalized image coordinates of the marker center
        u, v = marker_corners[0].mean(axis=0)
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # depth hint from the known marker size (optional extra: a vision
        # module that cannot estimate distance publishes z = 0 instead)
        _, tvecs, _ = aruco.estimatePoseSingleMarkers(
            [marker_corners], self.marker_length,
            self.camera_matrix, self.dist_coeffs)
        depth = float(tvecs[0][0][2])

        msg = PointStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = header.frame_id
        msg.point.x = (u - cx) / fx
        msg.point.y = (v - cy) / fy
        msg.point.z = depth
        self.point_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('aruco_detector')
    try:
        ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
