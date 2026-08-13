#!/usr/bin/env python3

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

        # Intrinsics of the PiOS camera (pixel position -> normalized offset).
        self.fx = float(rospy.get_param('~fx', 600.0))
        self.fy = float(rospy.get_param('~fy', 600.0))
        # Image center / principal point [px]. Set to 0 if PiOS already sends
        # center-relative errors (error_x/error_y) instead of absolute px/py.
        self.cx = float(rospy.get_param('~cx', 0.0))
        self.cy = float(rospy.get_param('~cy', 0.0))
        self.frame_id = rospy.get_param('~frame_id', 'camera')

        if self.fx == 0.0 or self.fy == 0.0:
            rospy.logfatal("udp_target_receiver: fx/fy must be non-zero")
            raise rospy.ROSInitException("fx/fy must be non-zero")
        if self.cx == 0.0 or self.cy == 0.0:
            rospy.logwarn("udp_target_receiver: cx/cy are 0 -- treating incoming "
                          "px/py as ALREADY center-relative. Set ~cx/~cy to the "
                          "image center if PiOS sends absolute pixel positions.")

        self.point_pub = rospy.Publisher('ibvs/target_point', PointStamped, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.settimeout(timeout)

        rospy.loginfo("udp_target_receiver: bound to %s:%d, fx=%.1f fy=%.1f "
                      "cx=%.1f cy=%.1f", ip, port, self.fx, self.fy, self.cx, self.cy)

    def parse(self, data):
        """JSON datagram -> (error_x_px, error_y_px), center-relative.

        Accepts absolute pixel positions (px/py, offset by cx/cy) or already
        center-relative errors (error_x/error_y). Raises on missing/bad data.
        """
        d = json.loads(data.decode('utf-8'))
        if 'px' in d or 'py' in d:
            error_x = float(d.get('px', 0.0)) - self.cx
            error_y = float(d.get('py', 0.0)) - self.cy
        elif 'error_x' in d or 'error_y' in d:
            error_x = float(d.get('error_x', 0.0))
            error_y = float(d.get('error_y', 0.0))
        else:
            raise KeyError("packet has neither px/py nor error_x/error_y")
        return error_x, error_y

    def publish(self, error_x, error_y):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.point.x = error_x / self.fx
        msg.point.y = error_y / self.fy
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
                error_x, error_y = self.parse(data)
            except (ValueError, KeyError, TypeError) as exc:
                rospy.logwarn_throttle(5.0, "udp_target_receiver: bad packet: %s" % exc)
                continue

            self.publish(error_x, error_y)

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
