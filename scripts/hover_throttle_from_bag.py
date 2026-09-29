#!/usr/bin/env python3
"""Measure the vehicle's real hover throttle from a flight bag.

    ./hover_throttle_from_bag.py [BAG ...]      (default: newest bag in
                                                 startup/optitrack/)

This is the number ~raw_hover_throttle (thrust_mode 'raw') must be set to.
MOT_THST_HOVER on the FCU is NOT a substitute: on this vehicle it read 0.18
while the 2026-09-25 bags hovered at 0.13-0.15.

HOVER = armed, at least MIN_HEIGHT above the lowest OptiTrack z seen while
armed (i.e. off the ground), and OptiTrack vertical speed below MAX_VZ, held
for MIN_SEGMENT seconds. Tilt is not filtered: at the few degrees these
flights use, cos(tilt) changes the needed throttle by well under 1%.

Per hover sample it records:
  vfr_hud.throttle   what ArduPilot actually output (0..1, the SAME scale a
                     GUID_OPTIONS 8 raw-thrust setpoint uses) -- the main answer
  setpoint thrust    what ibvs_controller sent, only meaningful in GUIDED_NOGPS
                     with thrust_mode 'raw' (should then match vfr_hud)
  battery voltage    hover throttle rises as the pack sags, so it is reported
                     per voltage band
"""

import glob
import os
import sys

import numpy as np
import rosbag

NS = '/red'
MIN_HEIGHT = 0.30      # [m] above the armed ground level
MAX_VZ = 0.05          # [m/s] |OptiTrack vz| (1 s smoothed) to count as hover
MIN_SEGMENT = 1.0      # [s] hover must last this long to be used

T_ODOM = NS + '/vrpn_client/estimated_odometry'
T_HUD = NS + '/mavros/vfr_hud'
T_STATE = NS + '/mavros/state'
T_BATT = NS + '/mavros/battery'
T_SP = NS + '/mavros/setpoint_raw/attitude'


def series(rows):
    return np.array(rows) if rows else np.zeros((0, 2))


def step_lookup(arr, t):
    """Value of a step signal (time, value) at times t (last value before)."""
    idx = np.searchsorted(arr[:, 0], t, side='right') - 1
    return idx


def analyse(path):
    odom, hud, batt, sp, state = [], [], [], [], []
    for topic, m, t in rosbag.Bag(path).read_messages(
            topics=[T_ODOM, T_HUD, T_STATE, T_BATT, T_SP]):
        ts = t.to_sec()
        if topic == T_ODOM:
            odom.append((ts, m.pose.pose.position.z))
        elif topic == T_HUD:
            hud.append((ts, m.throttle))
        elif topic == T_BATT:
            batt.append((ts, m.voltage))
        elif topic == T_SP:
            sp.append((ts, m.thrust))
        else:
            state.append((ts, m.armed, m.mode))

    name = os.path.basename(path)
    if not odom or not hud or not state:
        print('%s: missing OptiTrack, vfr_hud or mavros/state -- skipped' % name)
        return None
    odom, hud, batt, sp = map(series, (odom, hud, batt, sp))
    t0 = odom[0, 0]

    # OptiTrack on a 20 Hz grid; vz from a 1 s centred difference
    grid = np.arange(odom[0, 0], odom[-1, 0], 0.05)
    z = np.interp(grid, odom[:, 0], odom[:, 1])
    vz = np.zeros_like(z)
    vz[10:-10] = (z[20:] - z[:-20]) / 1.0

    st_t = np.array([s[0] for s in state])
    si = np.clip(step_lookup(st_t[:, None], grid), 0, len(state) - 1)
    armed = np.array([state[i][1] for i in si])
    mode = np.array([state[i][2] for i in si])
    if not armed.any():
        print('%s: never armed -- skipped' % name)
        return None
    ground = np.percentile(z[armed], 2)

    hover = armed & (z > ground + MIN_HEIGHT) & (np.abs(vz) < MAX_VZ)
    # keep only runs of at least MIN_SEGMENT
    keep = np.zeros_like(hover)
    i = 0
    while i < len(hover):
        if hover[i]:
            j = i
            while j < len(hover) and hover[j] and mode[j] == mode[i]:
                j += 1
            if (j - i) * 0.05 >= MIN_SEGMENT:
                keep[i:j] = True
            i = j
        else:
            i += 1

    thr = np.interp(grid, hud[:, 0], hud[:, 1])
    volt = np.interp(grid, batt[:, 0], batt[:, 1]) if len(batt) else np.full_like(grid, np.nan)
    spt = np.interp(grid, sp[:, 0], sp[:, 1]) if len(sp) else np.full_like(grid, np.nan)

    print('\n== %s   (ground z %.2f m, %.0f s hovering of %.0f s armed)'
          % (name, ground, keep.sum() * 0.05, armed.sum() * 0.05))
    if not keep.any():
        print('   no hover found (needs >%.2f m up, |vz| < %.2f m/s for %.0f s)'
              % (MIN_HEIGHT, MAX_VZ, MIN_SEGMENT))
        return None
    for md in sorted(set(mode[keep])):
        k = keep & (mode == md)
        line = ('   %-13s hover throttle (vfr_hud)  median %.3f  [p10 %.3f  p90 %.3f]  %4.0f s'
                % (md, np.median(thr[k]), np.percentile(thr[k], 10),
                   np.percentile(thr[k], 90), k.sum() * 0.05))
        if md == 'GUIDED_NOGPS' and not np.isnan(spt[k]).all():
            line += '   | we sent median %.3f' % np.nanmedian(spt[k])
        print(line)
    if not np.isnan(volt[keep]).all():
        edges = np.arange(np.floor(np.nanmin(volt[keep]) * 5) / 5,
                          np.nanmax(volt[keep]) + 0.2, 0.2)
        for lo, hi in zip(edges[:-1], edges[1:]):
            k = keep & (volt >= lo) & (volt < hi)
            if k.sum() * 0.05 >= MIN_SEGMENT:
                print('   battery %.1f-%.1f V   hover throttle median %.3f  (%3.0f s)'
                      % (lo, hi, np.median(thr[k]), k.sum() * 0.05))
    return thr[keep]


def main():
    bags = sys.argv[1:]
    if not bags:
        here = os.path.dirname(os.path.abspath(__file__))
        found = glob.glob(os.path.join(here, '..', 'startup', 'optitrack', '*.bag'))
        if not found:
            sys.exit('no bag given and none in startup/optitrack/')
        bags = [max(found, key=os.path.getmtime)]
    allthr = [a for a in (analyse(b) for b in bags) if a is not None]
    if len(allthr) > 1:
        a = np.concatenate(allthr)
        print('\nALL BAGS: hover throttle median %.3f  [p10 %.3f  p90 %.3f]'
              % (np.median(a), np.percentile(a, 10), np.percentile(a, 90)))
    if allthr:
        print('\n-> set raw_hover_throttle to the median above (fresh-battery value'
              ' if you fly full packs); the pid_z_raw I term trims the rest.')


if __name__ == '__main__':
    main()
