#!/usr/bin/env python

"""
ArUco vision module for the IBVS controller.

This node is ONE possible vision module: it detects an ArUco marker and
publishes the pixel coordinates of its center for the controller to steer
toward. Replace it with any other detector that speaks the same interface
-- including a plain object detector with no camera calibration at all --
and the controller works unchanged.

VISION MODULE INTERFACE (topic `ibvs/target_point`, geometry_msgs/PointStamped):
    point.x  detected object's pixel column (u), integer-valued,
             0 .. image_width
    point.y  detected object's pixel row (v), integer-valued,
             0 .. image_height
    point.z  unused, always 0.0 -- no depth. See ibvs_controller.py for how
             it descends without one.

    Publish ONLY while the target is actually detected -- the controller
    treats fresh messages as "target in sight" (TAG_IN_SIGHT state). The
    controller's ~image_width/~image_height must be set (in the launch
    file) to this node's actual camera resolution.

The marker's pixel center is exact even if marker_length is miscalibrated
-- no intrinsics or depth estimation are needed at all.
"""

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image

import cv2
import cv2.aruco as aruco


class ArucoDetector:

    def __init__(self):
        self.marker_id = rospy.get_param('~marker_id', 0)
        # detection rate: frames arriving faster than this are skipped
        # (camera runs at 30 fps, detection at 15 Hz is plenty)
        self.process_rate = rospy.get_param('~process_rate', 15.0)
        dictionary_name = rospy.get_param('~dictionary', 'DICT_4X4_50')
        self.dictionary = aruco.Dictionary_get(getattr(aruco, dictionary_name))
        self.detector_params = aruco.DetectorParameters_create()
        self.last_processed = rospy.Time(0)

        self.bridge = CvBridge()

        self.point_pub = rospy.Publisher('ibvs/target_point', PointStamped, queue_size=1)
        self.debug_pub = rospy.Publisher('ibvs/debug_image', Image, queue_size=1)

        rospy.Subscriber('camera/color/image_raw', Image,
                         self.image_callback, queue_size=1, buff_size=2 ** 22)

        rospy.loginfo(
            "aruco_detector: vision module for %s id %d, %g Hz",
            dictionary_name, self.marker_id, self.process_rate)

    def image_callback(self, msg):
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
        # pixel coordinates of the marker center
        u, v = marker_corners[0].mean(axis=0)

        msg = PointStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = header.frame_id
        msg.point.x = float(round(u))
        msg.point.y = float(round(v))
        msg.point.z = 0.0
        self.point_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('aruco_detector')
    try:
        ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
