#!/usr/bin/env python3

"""
UDP vision module: PiOS pixel detection -> ibvs/target_point.

VISION MODULE INTERFACE (topic `ibvs/target_point`, geometry_msgs/PointStamped):
    point.x  horizontal PIXEL POSITION of the detection in the image,
             positive RIGHT, measured from the image origin (top-left)
    point.y  vertical PIXEL POSITION of the detection in the image,
             positive DOWN, measured from the image origin (top-left)
    point.z  apparent SIZE of the target in pixels if the packet has 'size'
             (e.g. ArUco side length, branch width), else 0.0. Only used for
             the controller's time-to-contact; no real size is needed.
    header.stamp  the time the FRAME was taken, when the packet says how old
             the frame was (optional field 'age', seconds, measured on the
             detector's own clock: send time - frame time); otherwise the
             arrival time. virtual_camera.py needs the frame time to use the
             attitude the camera actually had; the controller ignores it.

    Packet (JSON): {"px": ..., "py": ..., "age": ..., "size": ...}
                   ('age' and 'size' optional)

This node publishes the detected POINT, not an error: it forwards what the
detector saw and does no geometry at all. The controller owns the setpoint
(~target_x / ~target_y, defaulting to the image centre cx/cy) and forms the
error itself as `detection - target`.

If your detector already emits centre-relative values (error_x/error_y),
they are forwarded unchanged -- set the controller's ~target_x / ~target_y
to 0 in that case, so it does not subtract the centre a second time.

The message stays PointStamped (float64 fields); the values carried in it
are integral, so `rostopic echo` shows whole pixels and the commanded body
rate can be read straight off the detection.
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

        # No camera geometry here at all: neither the image centre (cx/cy)
        # nor the focal lengths (fx/fy). This node forwards the detected
        # pixel position; the controller holds the setpoint and the
        # intrinsics, so there is exactly one place where the centre is
        # defined and no chance of subtracting it twice.
        self.frame_id = rospy.get_param('~frame_id', 'camera')

        self.point_pub = rospy.Publisher('ibvs/target_point', PointStamped, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.settimeout(timeout)

        rospy.loginfo("udp_target_receiver: bound to %s:%d "
                      "(publishing raw detected PIXEL POSITION)", ip, port)

    def parse(self, data):
        """JSON datagram -> (px, py, age, size): the detection's WHOLE-pixel
        position, the frame's age in seconds (None if not sent) and the
        target's apparent size in pixels (0.0 if not sent).

        Accepts px/py (pixel position in the image) or error_x/error_y
        (already centre-relative); both are forwarded unchanged, the
        difference being what the controller's ~target_x/~target_y must be
        set to. Raises on missing/bad data. The result is rounded to int:
        the detection is a pixel index, and keeping it integral makes the
        published point directly comparable to what the detector reports.
        """
        d = json.loads(data.decode('utf-8'))
        if 'px' in d or 'py' in d:
            px = float(d.get('px', 0.0))
            py = float(d.get('py', 0.0))
        elif 'error_x' in d or 'error_y' in d:
            px = float(d.get('error_x', 0.0))
            py = float(d.get('error_y', 0.0))
        else:
            raise KeyError("packet has neither px/py nor error_x/error_y")
        age = None
        if 'age' in d:
            age = float(d['age'])
            if not 0.0 <= age < 1.0:
                # a frame age outside [0, 1) s is a detector clock/unit bug:
                # keep the detection, but do not trust its timing
                rospy.logwarn_throttle(
                    5.0, "udp_target_receiver: implausible frame age %.3f s "
                         "-- stamping with the arrival time" % age)
                age = None
        size = 0.0
        if 'size' in d:
            size = float(d['size'])
            if not size >= 0.0:      # also rejects NaN
                size = 0.0
        return int(round(px)), int(round(py)), age, size

    def publish(self, px, py, stamp, size=0.0):
        """Publish the detected pixel position -- no geometry applied."""
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.point.x = float(px)
        msg.point.y = float(py)
        msg.point.z = float(size)
        self.point_pub.publish(msg)

    def spin(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(65507)
                arrival = rospy.Time.now()
            except socket.timeout:
                continue
            except socket.error as exc:
                rospy.logerr_throttle(5.0, "udp_target_receiver: socket error: %s" % exc)
                continue

            try:
                px, py, age, size = self.parse(data)
            except (ValueError, KeyError, TypeError) as exc:
                rospy.logwarn_throttle(5.0, "udp_target_receiver: bad packet: %s" % exc)
                continue

            # frame time = arrival - age (the LAN hop itself is < 1 ms)
            stamp = arrival - rospy.Duration(age) if age else arrival
            self.publish(px, py, stamp, size)

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
