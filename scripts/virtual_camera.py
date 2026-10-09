#!/usr/bin/env python3

"""
Virtual level camera: removes the image motion caused by the vehicle's ROLL
and PITCH from the detected point, in real time, from the IMU attitude.

    ibvs/target_point   raw detection, pixels, stamped at FRAME time --+
                                                                       +--> ibvs/target_point_virtual
    mavros/imu/data     attitude, ~50 Hz, kept in a short buffer ------+    same pixels, as seen by a
                                                                            camera that never rolls or
                                                                            pitches (it keeps yaw)

WHY: the camera is rigidly mounted and the vehicle is underactuated -- it
must tilt to translate. Every tilt rotates the camera, which moves the target
in the image although the vehicle has not moved. On this vehicle that is
~20 px per degree (2026-09-29 bags), in the direction that makes the IBVS
loop tilt MORE: positive feedback, hardest on the D term. This node reports
where the target would appear in a camera that stays level.

THE MATH -- exact (no small-angle approximation) and nothing fitted:

    r_C = K^-1 [u, v, 1]^T                  bearing in the physical camera
    R_t = Rz(yaw)^T R_WB(t_frame)           attitude WITHOUT yaw = Ry(pitch) Rx(roll)
    r_V = R_BC^T R_t R_BC r_C               same bearing, virtual level camera
    u_V = fx * r_V.x / r_V.z + W/2          back to pixels in the virtual camera
    v_V = fy * r_V.y / r_V.z + H/2          (same focal length, centred)

    R_WB  world <- body, the IMU quaternion (ENU world / FLU body, as mavros
          publishes it)
    R_BC  body <- camera: HOW THE CAMERA IS MOUNTED (~camera/mount)
    K     the camera's intrinsics AT THE DETECTOR'S RESOLUTION (~camera/*):
          a property of the camera, not of the airframe

Small-angle check, down mount: u_V ~ u - fx*roll, v_V ~ v + fy*pitch. That
is exactly the px-per-degree seen in flight -- the formula predicts it, so
it never has to be measured.

What it needs, and nothing else: K, the mount, the IMU, and the time the
frame was taken. A new airframe changes only ~camera/mount (and K if the
camera changes).

TIMING: the attitude is taken at the message's header.stamp, interpolated
(SLERP) between buffered IMU samples. udp_target_receiver sets that stamp to
the FRAME time when the detector sends the frame's age (packet field 'age',
seconds); without it the stamp is the arrival time and the attitude used is
the detector latency (~50 ms) too late -- the node warns once.

VIRTUAL PRINCIPAL POINT: the output camera is centred (W/2, H/2), so the
virtual point sits at the image centre exactly when the target lies on the
virtual optical axis -- straight below (down mount) or above (up mount) the
camera along gravity. The controller aims at the image centre, so flying on
this point aligns the VEHICLE over the target instead of pointing the tilted
optical axis at it.

INPUT MUST BE ABSOLUTE PIXELS (detector packets with px/py). Centre-relative
error_x/error_y packets are not supported here.

No attitude for the frame time (IMU missing, stale or gapped) -> NOTHING is
published for that detection. With the controller on the virtual point
(launch arg compensate:=true) that reads as target not seen, never as an
uncompensated point.

Debug topic ibvs/virtual_camera/tilt (PointStamped):
    x = roll, y = pitch [rad] used for this frame,
    z = now - frame stamp [s] (processing + any age the detector reported)
"""

import bisect
import collections
import math

import numpy as np


# R_BC (body <- camera) for the mounts this vehicle has flown. Columns are the
# camera's x (image right), y (image down) and z (optical axis) expressed in
# the body FLU frame.
#   down: image right = body RIGHT, image down = body BACK, looking down.
#         Matches image_x_sign +1 / image_y_sign +1 and the flight data
#         (roll+ moves the tag right, pitch+ moves it up).
#   up:   image right = body LEFT,  image down = body BACK, looking up.
#         Matches image_x_sign -1 / image_y_sign +1 (bench, 2026-08-20).
MOUNTS = {
    'down': [[0.0, -1.0, 0.0],
             [-1.0, 0.0, 0.0],
             [0.0, 0.0, -1.0]],
    'up': [[0.0, -1.0, 0.0],
           [1.0, 0.0, 0.0],
           [0.0, 0.0, 1.0]],
}


# --------------------------------------------------------------------------
# Pure math (numpy only, no ROS) -- see test_virtual_camera.py
# --------------------------------------------------------------------------

def quat_to_matrix(q):
    """Rotation matrix of a unit quaternion given as [x, y, z, w] (ROS order)."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0, q1, f):
    """Spherical interpolation between [x, y, z, w] quaternions, f in [0, 1]."""
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:                       # take the short way round
        q1, d = -q1, -d
    if d > 0.9995:                    # nearly identical: linear is exact enough
        q = q0 + f * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(d)
    s = math.sin(theta)
    return (math.sin((1 - f) * theta) * q0 + math.sin(f * theta) * q1) / s


def tilt_only(R_WB):
    """Rz(yaw)^T R_WB: the attitude with its yaw removed, = Ry(pitch) Rx(roll)."""
    yaw = math.atan2(R_WB[1, 0], R_WB[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    Rz_T = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    return Rz_T.dot(R_WB)


def roll_pitch(R_t):
    """(roll, pitch) of a yaw-free rotation Ry(pitch) Rx(roll)."""
    return (math.atan2(R_t[2, 1], R_t[2, 2]),
            math.asin(max(-1.0, min(1.0, -R_t[2, 0]))))


def virtual_pixel(u, v, K, R_BC, R_WB, width, height):
    """Raw pixel (u, v) -> pixel in the virtual level camera, or None.

    K = (fx, fy, cx, cy) of the physical camera. The virtual camera has the
    same fx, fy and its principal point at the image centre. None only if
    the bearing ends up behind the virtual camera (impossible at any
    sensible tilt).
    """
    fx, fy, cx, cy = K
    r_C = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    r_V = R_BC.T.dot(tilt_only(R_WB)).dot(R_BC).dot(r_C)
    if r_V[2] <= 1e-6:
        return None
    return (fx * r_V[0] / r_V[2] + width / 2.0,
            fy * r_V[1] / r_V[2] + height / 2.0)


def intrinsics(width, height, fx=0.0, fy=0.0, cx=None, cy=None,
               hfov_deg=0.0, vfov_deg=0.0):
    """K = (fx, fy, cx, cy) from a calibration (fx > 0) or a datasheet FOV.

    Calibration values win. From the FOV: fx = (W/2) / tan(HFOV/2), and
    fy the same way from VFOV if given, else fy = fx (square pixels). The
    principal point defaults to the image centre.
    """
    if fx <= 0.0:
        if hfov_deg <= 0.0:
            raise ValueError("no camera intrinsics: set camera/fx, camera/fy "
                             "(calibration) or camera/hfov_deg (datasheet)")
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        if vfov_deg > 0.0:
            fy = (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    if fy <= 0.0:
        fy = fx
    hfov = math.degrees(2.0 * math.atan((width / 2.0) / fx))
    if not 10.0 < hfov < 170.0:
        raise ValueError("camera intrinsics imply an HFOV of %.1f deg at %d px "
                         "wide -- wrong fx or wrong image_width?" % (hfov, width))
    return (fx, fy,
            width / 2.0 if cx is None else cx,
            height / 2.0 if cy is None else cy)


def mount_matrix(mount, mission_mode='land', custom=None):
    """R_BC for ~camera/mount: 'down', 'up', 'auto' (perch -> up, else down)
    or 'custom' (custom = 9 numbers, row-major, a proper rotation)."""
    if mount == 'auto':
        mount = 'up' if mission_mode == 'perch' else 'down'
    if mount == 'custom':
        if custom is None or len(custom) != 9:
            raise ValueError("camera/mount 'custom' needs camera/R_BC (9 numbers)")
        R = np.array(custom, dtype=float).reshape(3, 3)
    elif mount in MOUNTS:
        R = np.array(MOUNTS[mount])
    else:
        raise ValueError("unknown camera/mount '%s'" % mount)
    if (not np.allclose(R.T.dot(R), np.eye(3), atol=1e-6)
            or abs(np.linalg.det(R) - 1.0) > 1e-6):
        raise ValueError("camera R_BC is not a proper rotation")
    return R, mount


class ImuBuffer:
    """Last few hundred ms of IMU attitude, looked up at arbitrary times."""

    def __init__(self, length_s=1.0, max_gap_s=0.1):
        self.length = length_s
        self.max_gap = max_gap_s
        self.samples = collections.deque()          # (t [s], q [x,y,z,w])

    def add(self, t, q):
        self.samples.append((t, np.asarray(q, dtype=float)))
        while self.samples and self.samples[0][0] < t - self.length:
            self.samples.popleft()

    def attitude_at(self, t):
        """Quaternion at time t, or None if the buffer cannot vouch for it.

        Interpolated between the samples around t. A frame newer than the
        last sample uses that sample only if it is within max_gap; a gap
        between samples longer than max_gap (IMU dropout) is never bridged.
        """
        s = list(self.samples)
        if not s:
            return None
        if t >= s[-1][0]:
            return s[-1][1] if t - s[-1][0] <= self.max_gap else None
        if t < s[0][0]:
            return None
        i = bisect.bisect_right([x[0] for x in s], t)
        (t0, q0), (t1, q1) = s[i - 1], s[i]
        if t1 - t0 > self.max_gap:
            return None
        return slerp(q0, q1, (t - t0) / (t1 - t0) if t1 > t0 else 0.0)


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------

class VirtualCamera:

    def __init__(self):
        import rospy
        from geometry_msgs.msg import PointStamped
        from sensor_msgs.msg import Imu
        self.rospy = rospy
        self.PointStamped = PointStamped

        self.width = rospy.get_param('~image_width', 1280)
        self.height = rospy.get_param('~image_height', 720)
        try:
            self.K = intrinsics(
                self.width, self.height,
                fx=rospy.get_param('~camera/fx', 0.0),
                fy=rospy.get_param('~camera/fy', 0.0),
                cx=rospy.get_param('~camera/cx', None),
                cy=rospy.get_param('~camera/cy', None),
                hfov_deg=rospy.get_param('~camera/hfov_deg', 0.0),
                vfov_deg=rospy.get_param('~camera/vfov_deg', 0.0))
            self.R_BC, mount = mount_matrix(
                rospy.get_param('~camera/mount', 'auto'),
                rospy.get_param('~mission_mode', 'land'),
                rospy.get_param('~camera/R_BC', None))
        except ValueError as exc:
            raise rospy.ROSInitException("virtual_camera: %s" % exc)

        self.imu = ImuBuffer(rospy.get_param('~imu_buffer_s', 1.0),
                             rospy.get_param('~imu_max_gap_s', 0.1))
        self.warned_no_age = False

        self.point_pub = rospy.Publisher(
            'ibvs/target_point_virtual', PointStamped, queue_size=1)
        self.tilt_pub = rospy.Publisher(
            'ibvs/virtual_camera/tilt', PointStamped, queue_size=1)
        rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback,
                         queue_size=50)
        rospy.Subscriber('ibvs/target_point', PointStamped,
                         self.target_callback, queue_size=1)

        fx, fy, cx, cy = self.K
        rospy.loginfo("virtual_camera: mount '%s', %dx%d, fx %.1f fy %.1f "
                      "cx %.1f cy %.1f (HFOV %.1f deg)", mount, self.width,
                      self.height, fx, fy, cx, cy,
                      math.degrees(2 * math.atan(self.width / 2.0 / fx)))

    def imu_callback(self, msg):
        o = msg.orientation
        self.imu.add(msg.header.stamp.to_sec(), [o.x, o.y, o.z, o.w])

    def target_callback(self, msg):
        rospy = self.rospy
        stamp = msg.header.stamp
        age = (rospy.Time.now() - stamp).to_sec()
        if age < 0.005 and not self.warned_no_age:
            # the stamp is (almost) the arrival time: the detector sends no
            # frame age, so the attitude used is the detector latency late
            self.warned_no_age = True
            rospy.logwarn("virtual_camera: detections carry no frame age "
                          "('age' in the UDP packet) -- using the attitude at "
                          "ARRIVAL, i.e. late by the detector latency")

        q = self.imu.attitude_at(stamp.to_sec())
        if q is None:
            rospy.logwarn_throttle(
                2.0, "virtual_camera: no IMU attitude for the frame time -- "
                     "detection DROPPED (is mavros/imu/data streaming?)")
            return
        R_WB = quat_to_matrix(q)
        out = virtual_pixel(msg.point.x, msg.point.y, self.K, self.R_BC,
                            R_WB, self.width, self.height)
        if out is None:
            return

        vm = self.PointStamped()
        vm.header.stamp = stamp
        vm.header.frame_id = 'virtual_camera'
        vm.point.x, vm.point.y = out
        vm.point.z = msg.point.z      # apparent size [px], passed through
        self.point_pub.publish(vm)

        roll, pitch = roll_pitch(tilt_only(R_WB))
        tm = self.PointStamped()
        tm.header.stamp = stamp
        tm.point.x = roll
        tm.point.y = pitch
        tm.point.z = age
        self.tilt_pub.publish(tm)


if __name__ == '__main__':
    import rospy
    rospy.init_node('virtual_camera')
    try:
        VirtualCamera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
