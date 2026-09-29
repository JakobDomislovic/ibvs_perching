#!/usr/bin/env python3
"""Tests for virtual_camera.py's math -- no ROS master needed.

    python3 scripts/test_virtual_camera.py

The key test builds the image INDEPENDENTLY: a target at a known 3-D point,
projected into the tilted physical camera (what the detector would see) and
into a level camera with the same yaw (what the virtual camera must report).
virtual_pixel() applied to the first must give the second, for any attitude.
"""

import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import virtual_camera as vc  # noqa: E402

W, H = 1280, 720
# deliberately off-centre principal point and fx != fy, to test those paths
K = (1024.0, 1010.0, 652.0, 351.0)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def project(p_C, fx, fy, cx, cy):
    return fx * p_C[0] / p_C[2] + cx, fy * p_C[1] / p_C[2] + cy


def check(name, ok):
    print('%-62s %s' % (name, 'OK' if ok else 'FAIL'))
    if not ok:
        sys.exit(1)


def test_exact_against_independent_projection():
    random.seed(0)
    worst = 0.0
    for mount, below in (('down', -1.0), ('up', 1.0)):
        R_BC = np.array(vc.MOUNTS[mount])
        for _ in range(2000):
            roll = math.radians(random.uniform(-15, 15))
            pitch = math.radians(random.uniform(-15, 15))
            yaw = math.radians(random.uniform(-180, 180))
            R_WB = rot_z(yaw).dot(rot_y(pitch)).dot(rot_x(roll))
            # target 1-3 m below (down) / above (up), offset sideways
            P = np.array([random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4),
                          below * random.uniform(1.0, 3.0)])
            # what the tilted physical camera sees
            p_C = R_BC.T.dot(R_WB.T.dot(P))
            if p_C[2] <= 0.1:
                continue
            u, v = project(p_C, *K)
            # what a level camera (same yaw, centred principal point) sees
            p_Cv = R_BC.T.dot(rot_z(yaw).T.dot(P))
            u_exp, v_exp = project(p_Cv, K[0], K[1], W / 2.0, H / 2.0)
            u_V, v_V = vc.virtual_pixel(u, v, K, R_BC, R_WB, W, H)
            worst = max(worst, abs(u_V - u_exp), abs(v_V - v_exp))
    check('virtual pixel == level-camera projection (2x2000 attitudes)',
          worst < 1e-6)


def test_signs_match_flight_data():
    """Down mount, target straight below: +roll moved the tag RIGHT and
    +pitch moved it UP in the 2026-09-29 bags. The raw projection must
    reproduce that, and the virtual point must stay at the centre."""
    R_BC = np.array(vc.MOUNTS['down'])
    P = np.array([0.0, 0.0, -1.5])
    Kc = (1024.0, 1024.0, W / 2.0, H / 2.0)
    for axis, R_WB in (('roll', rot_x(math.radians(2))),
                       ('pitch', rot_y(math.radians(2)))):
        u, v = project(R_BC.T.dot(R_WB.T.dot(P)), *Kc)
        u_V, v_V = vc.virtual_pixel(u, v, Kc, R_BC, R_WB, W, H)
        if axis == 'roll':
            raw_ok = u > W / 2.0 + 30 and abs(v - H / 2.0) < 1e-6
        else:
            raw_ok = v < H / 2.0 - 30 and abs(u - W / 2.0) < 1e-6
        check('down, +2 deg %-5s: raw moves as in flight' % axis, raw_ok)
        check('down, +2 deg %-5s: virtual stays at centre' % axis,
              abs(u_V - W / 2.0) < 1e-6 and abs(v_V - H / 2.0) < 1e-6)
    # small-angle form quoted in the docstring: u_V ~ u - fx*roll
    roll = math.radians(1.0)
    u, v = project(R_BC.T.dot(rot_x(roll).T.dot(P)), *Kc)
    check('small angle: raw shift = fx*tan(roll) (%.1f px/deg at fx 1024)'
          % (u - W / 2.0), abs((u - W / 2.0) - 1024.0 * math.tan(roll)) < 1e-6)


def test_level_is_identity_up_to_centring():
    R_BC = np.array(vc.MOUNTS['down'])
    u_V, v_V = vc.virtual_pixel(900.0, 200.0, K, R_BC, np.eye(3), W, H)
    check('level vehicle: only the principal point moves to the centre',
          abs(u_V - (900.0 - K[2] + W / 2.0)) < 1e-9 and
          abs(v_V - (200.0 - K[3] + H / 2.0)) < 1e-9)


def test_intrinsics_and_mount():
    fx, fy, cx, cy = vc.intrinsics(W, H, hfov_deg=64.0)
    check('HFOV 64 deg at 1280 px -> fx = %.1f px, square pixels' % fx,
          abs(fx - 640.0 / math.tan(math.radians(32.0))) < 1e-9 and fy == fx
          and (cx, cy) == (W / 2.0, H / 2.0))
    check('calibration fx/fy/cx/cy override the FOV',
          vc.intrinsics(W, H, fx=1100.0, fy=1090.0, cx=650.0, cy=355.0,
                        hfov_deg=64.0) == (1100.0, 1090.0, 650.0, 355.0))
    for bad in (dict(), dict(fx=40.0)):
        try:
            vc.intrinsics(W, H, **bad)
            ok = False
        except ValueError:
            ok = True
        check('rejects intrinsics %s' % (bad or 'missing'), ok)
    check("mount 'auto': perch -> up, land/hover -> down",
          vc.mount_matrix('auto', 'perch')[1] == 'up' and
          vc.mount_matrix('auto', 'hover')[1] == 'down')
    try:
        vc.mount_matrix('custom', custom=[1, 0, 0, 0, 1, 0, 0, 0, -1])
        ok = False
    except ValueError:
        ok = True
    check('rejects a custom R_BC that is a reflection', ok)


def test_imu_buffer():
    b = vc.ImuBuffer(length_s=1.0, max_gap_s=0.1)
    q = lambda r: np.array([math.sin(r / 2), 0.0, 0.0, math.cos(r / 2)])
    for i in range(51):                       # 50 Hz, roll ramps 0 -> 10 deg
        b.add(i * 0.02, q(math.radians(0.2 * i)))
    roll = lambda qq: vc.roll_pitch(vc.quat_to_matrix(qq))[0]
    check('SLERP between samples', abs(math.degrees(
        roll(b.attitude_at(0.51))) - 5.1) < 1e-6)
    check('frame just after the last sample -> last sample',
          b.attitude_at(1.05) is not None)
    check('frame > max_gap after the last sample -> None (stale IMU)',
          b.attitude_at(1.2) is None)
    check('frame older than the buffer -> None', b.attitude_at(-0.5) is None)
    b.add(1.5, q(0.0))                        # 0.5 s dropout
    check('never interpolates across an IMU dropout', b.attitude_at(1.3) is None)


if __name__ == '__main__':
    test_exact_against_independent_projection()
    test_signs_match_flight_data()
    test_level_is_identity_up_to_centring()
    test_intrinsics_and_mount()
    test_imu_buffer()
    print('all tests passed')
