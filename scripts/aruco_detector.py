#!/usr/bin/env python

"""
Real ArUco tag detector for IBVS perching.

Subscribes to the down-facing camera, detects a single ArUco marker
(DICT_4X4_50 id 0 by default, 30 cm side), estimates its metric pose from
the camera intrinsics, transforms it into the body FLU frame and publishes
it on ibvs/tag_pose -- the exact same interface the mock publisher used, so
the controller needs no changes.

Frames:
    optical (OpenCV/aruco): x right in image, y down in image, z out of lens
    camera mount (URDF)   : camera_box oriented by ~camera_rpy, then the
                            fixed camera_help joint rpy=(-1.5708, 0, -1.5708)
                            turns camera_box into the optical frame
    body FLU              : x forward, y left, z up

    tag_body = R_body_optical * tvec + camera_xyz

With the down-facing mount (rpy = -1.5708, 1.5708, 0) this works out to
tag_body = (t_opt.x, -t_opt.y, -t_opt.z) + mount offset, i.e. a tag below
the vehicle has negative body z, as the controller expects (target_z < 0).

No attitude compensation is applied: at the tilt angles this controller
commands (max_tilt ~8.5 deg) the small-angle error is well inside the
alignment tolerance.
"""

import numpy as np

import rospy
import tf.transformations as tft
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
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

        # camera mount on the body, same values as the URDF camera_joint
        camera_xyz = rospy.get_param('~camera_xyz', [0.0, 0.0, -0.05])
        camera_rpy = rospy.get_param('~camera_rpy', [-1.570796, 1.570796, 0.0])

        # body <- camera_box <- optical (camera_help joint in the URDF)
        r_body_box = tft.euler_matrix(*camera_rpy)[:3, :3]
        r_box_optical = tft.euler_matrix(-1.570796, 0.0, -1.570796)[:3, :3]
        self.r_body_optical = np.dot(r_body_box, r_box_optical)
        self.t_body_camera = np.array(camera_xyz)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()

        self.pose_pub = rospy.Publisher('ibvs/tag_pose', PoseStamped, queue_size=1)
        self.debug_pub = rospy.Publisher('ibvs/debug_image', Image, queue_size=1)

        rospy.Subscriber('camera/color/camera_info', CameraInfo,
                         self.camera_info_callback, queue_size=1)
        rospy.Subscriber('camera/color/image_raw', Image,
                         self.image_callback, queue_size=1, buff_size=2 ** 22)

        rospy.loginfo(
            "aruco_detector: looking for %s id %d (%.2f m), camera mount xyz=%s",
            dictionary_name, self.marker_id, self.marker_length, camera_xyz)

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

        tvec_optical = None
        rmat_optical = None
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                if marker_id != self.marker_id:
                    continue
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    [marker_corners], self.marker_length,
                    self.camera_matrix, self.dist_coeffs)
                tvec_optical = tvecs[0][0]
                rmat_optical, _ = cv2.Rodrigues(rvecs[0][0])
                break

        if tvec_optical is not None:
            self.publish_pose(msg.header.stamp, tvec_optical, rmat_optical)

        if self.debug_pub.get_num_connections() > 0:
            debug = frame.copy()
            if ids is not None:
                aruco.drawDetectedMarkers(debug, corners, ids)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def publish_pose(self, stamp, tvec_optical, rmat_optical):
        position_body = np.dot(self.r_body_optical, tvec_optical) + self.t_body_camera

        rmat_body = np.eye(4)
        rmat_body[:3, :3] = np.dot(self.r_body_optical, rmat_optical)
        quat = tft.quaternion_from_matrix(rmat_body)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = position_body[0]
        msg.pose.position.y = position_body[1]
        msg.pose.position.z = position_body[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('aruco_detector')
    try:
        ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
