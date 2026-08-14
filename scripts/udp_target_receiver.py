#!/usr/bin/env python3

"""
UDP vision module for the IBVS controller.

The companion computer (PiOS) detects the target and streams its pixel
position over UDP as a JSON datagram; this node republishes it as
ibvs/target_point (geometry_msgs/PointStamped), speaking the SAME
calibration-free interface as aruco_detector.py -- see the VISION MODULE
INTERFACE in ibvs_controller.py:
    point.x  pixel column (u), 0 .. image_width
    point.y  pixel row (v), 0 .. image_height
    point.z  unused, always 0.0

No camera intrinsics are needed: the controller normalizes by
image_width/image_height itself (set here and on the controller, together,
in the launch file), exactly like the on-board ArUco detector.
"""

import json
import socket

import rospy
from geometry_msgs.msg import PointStamped


class UdpTargetReceiver:

    def __init__(self):
        rospy.init_node('udp_target_receiver')

        ip = rospy.get_param('~bind_ip', '0.0.0.0')
        port = int(rospy.get_param('~bind_port', 5005))
        timeout = float(rospy.get_param('~timeout', 0.5))

        # Must match the resolution the PiOS detector reports px/py in (if it
        # downscales before detection, use the DOWNSCALED size here, not the
        # capture size), and must match the controller's ~image_width /
        # ~image_height (set once, together, in the launch file).
        self.image_width = float(rospy.get_param('~image_width', 640))
        self.image_height = float(rospy.get_param('~image_height', 480))
        self.frame_id = rospy.get_param('~frame_id', 'camera')

        self.point_pub = rospy.Publisher('ibvs/target_point', PointStamped, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.settimeout(timeout)

        rospy.loginfo("udp_target_receiver: bound to %s:%d, image %dx%d",
                      ip, port, self.image_width, self.image_height)

    def parse(self, data):
        """JSON datagram -> (u, v), absolute pixel coordinates.

        Accepts absolute pixel positions (px/py) directly, or
        center-relative pixel offsets (error_x/error_y) which are shifted
        back to absolute using image_width/image_height. Raises on
        missing/bad data.
        """
        d = json.loads(data.decode('utf-8'))
        if 'px' in d or 'py' in d:
            u = float(d.get('px', 0.0))
            v = float(d.get('py', 0.0))
        elif 'error_x' in d or 'error_y' in d:
            u = self.image_width / 2.0 + float(d.get('error_x', 0.0))
            v = self.image_height / 2.0 + float(d.get('error_y', 0.0))
        else:
            raise KeyError("packet has neither px/py nor error_x/error_y")
        return u, v

    def publish(self, u, v):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.point.x = u
        msg.point.y = v
        msg.point.z = 0.0
        self.point_pub.publish(msg)

    def spin(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(65507)
            except socket.timeout:
                continue
            except socket.error as exc:
                rospy.logerr_throttle(5.0, "udp_target_receiver: socket error: %s" % exc)
                continue

            try:
                u, v = self.parse(data)
            except (ValueError, KeyError, TypeError) as exc:
                rospy.logwarn_throttle(5.0, "udp_target_receiver: bad packet: %s" % exc)
                continue

            self.publish(u, v)

    def shutdown(self):
        self.sock.close()


def main():
    node = UdpTargetReceiver()
    rospy.on_shutdown(node.shutdown)
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
