#!/usr/bin/env python
"""
Offline simulator for the IBVS roll/pitch command.

Reproduces EXACTLY what ibvs_controller.py computes for a given detection
pixel, using the params from custom_config/ibvs_params_rw.yaml, so you can
check the image_x_sign / image_y_sign polarity and the commanded tilt
without flying.

Mirrors, in order:
    target_callback()      pixel -> normalized error (both axes / half_w)
    compute_desired_tilt() PID on -err, clamped to +-max_tilt
    publish_setpoint()     tilt -> what actually goes to the FCU

Usage
-----
    ./sim_tilt.py                      interactive: type "px py" per line
    ./sim_tilt.py 900 200              one-shot, single detection
    ./sim_tilt.py --sweep              polarity table: is the sign right?
    ./sim_tilt.py --rate               also show RATE-mode body rates
    ./sim_tilt.py --reset              zero the PID state between inputs
                                       (default: state CARRIES OVER, like
                                        the real loop, so the I-term winds up)

Interactive commands: 'r' reset PIDs, 'q' quit.

NOTE ON RATES: with command_mode 'attitude' (the configured value) the
controller sends an ANGLE, not a rate -- the body_rate fields are ignored
(type_mask 7) and are legitimately zero on the wire. Body rates are shown
only under --rate, which simulates command_mode 'rate'.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, 'custom_config', 'ibvs_params_rw.yaml')

# From start_udp.sh -- the launch passes these, they are NOT in the yaml.
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720


def clamp(v, lo, hi):
    """Identical to ibvs_controller.clamp."""
    return max(lo, min(hi, v))


def load_params():
    """Read the real config. yaml if available, else a tiny fallback parser."""
    try:
        import yaml
        with open(CONFIG) as f:
            return yaml.safe_load(f)
    except ImportError:
        pass

    # Fallback: flat "key: value" + one level of nesting (pid_xy), comments
    # stripped. Enough for this file's structure.
    params, section = {}, None
    with open(CONFIG) as f:
        for line in f:
            raw = line.rstrip('\n')
            if '#' in raw:
                raw = raw[:raw.index('#')]
            if not raw.strip():
                continue
            indented = raw[0] in ' \t'
            key, _, val = raw.strip().partition(':')
            val = val.strip()
            if not val:                       # section header, e.g. "pid_xy:"
                section = key.strip()
                params[section] = {}
                continue
            try:
                val = float(val)
            except ValueError:
                pass
            if indented and section:
                params[section][key.strip()] = val
            else:
                section = None
                params[key.strip()] = val
    return params


class Pid(object):
    """Verbatim copy of ibvs_controller.Pid (integral clamped by contribution)."""

    def __init__(self, kp, ki, kd, out_min, out_max, i_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.i_max = i_max
        self.integral = 0.0
        self.p_term = self.i_term = self.d_term = 0.0

    def reset(self):
        self.integral = 0.0
        self.p_term = self.i_term = self.d_term = 0.0

    def update(self, error, error_dot, dt):
        i_term = 0.0
        if self.ki > 0.0:
            self.integral += error * dt
            self.integral = clamp(self.integral,
                                  -self.i_max / self.ki, self.i_max / self.ki)
            i_term = self.ki * self.integral

        self.p_term = self.kp * error
        self.i_term = i_term
        self.d_term = self.kd * error_dot

        out = self.p_term + self.i_term + self.d_term
        return clamp(out, self.out_min, self.out_max)


class TiltSim(object):
    """The controller's x-y chain, minus ROS."""

    def __init__(self, p):
        self.image_width = IMAGE_WIDTH
        self.image_height = IMAGE_HEIGHT
        self.half_w = self.image_width / 2.0
        self.half_h = self.image_height / 2.0

        self.mission_mode = p.get('mission_mode', 'land')
        self.image_x_sign = float(p.get('image_x_sign', 1.0))
        self.image_y_sign = float(p.get('image_y_sign', 1.0))
        self.target_x = float(p.get('target_x', 0.0))
        self.target_y = float(p.get('target_y', 0.0))
        self.max_tilt = float(p.get('max_tilt', 0.15))
        self.control_rate = float(p.get('control_rate', 20.0))
        self.dt = 1.0 / self.control_rate
        self.command_mode = p.get('command_mode', 'attitude')
        self.kp_att = float(p.get('kp_att', 1.5))
        self.max_body_rate = float(p.get('max_body_rate', 0.35))

        self.align_tolerance_px = float(p.get('align_tolerance_px', 70))
        self.descend_xy_gate_px = float(p.get('descend_xy_gate_px', 70))
        self.hover_thrust = float(p.get('hover_thrust', 0.5))
        self.thrust_max = float(p.get('thrust_max', 0.6))
        self.thrust_min = float(p.get('thrust_min', 0.45))
        self.perch_climb_thrust = float(p.get('perch_climb_thrust', 0.52))
        self.land_descend_thrust = float(p.get('land_descend_thrust', 0.45))

        pid = p.get('pid_xy', {})
        self.d_filter = float(pid.get('d_filter', 1.0))
        self.d_max_dt = float(pid.get('d_max_dt', 0.4))
        args = (float(pid.get('kp', 0.0)), float(pid.get('ki', 0.0)),
                float(pid.get('kd', 0.0)),
                -self.max_tilt, self.max_tilt, float(pid.get('i_max', 0.05)))
        self.pid_x = Pid(*args)
        self.pid_y = Pid(*args)

        # aim point in the pixel frame (ibvs_controller.__init__ 422-423)
        self.aim_px_x = self.target_x * self.half_w * self.image_x_sign
        self.aim_px_y = -self.target_y * self.half_w * self.image_y_sign

        self.t_x = self.t_y = None
        self.t_x_dot = self.t_y_dot = 0.0
        self.err_px = None
        self.t_now = 0.0

    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.t_x = self.t_y = None
        self.t_x_dot = self.t_y_dot = 0.0
        self.err_px = None

    def target_callback(self, px, py, dt=None):
        """Pixel -> normalized error. NOTE both axes divide by half_w."""
        norm_x = (px - self.half_w) / self.half_w
        norm_y = (py - self.half_h) / self.half_w

        t_x = self.image_x_sign * norm_x
        t_y = -self.image_y_sign * norm_y

        if dt is None:
            dt = self.dt
        if self.t_x is not None:
            if dt > self.d_max_dt:
                self.t_x_dot = self.t_y_dot = 0.0
            elif dt > 1e-3:
                a = self.d_filter
                self.t_x_dot = (1.0 - a) * self.t_x_dot + a * (t_x - self.t_x) / dt
                self.t_y_dot = (1.0 - a) * self.t_y_dot + a * (t_y - self.t_y) / dt

        self.t_x, self.t_y = t_x, t_y
        self.err_px = (px - self.half_w, py - self.half_h)

    def lateral_error_px(self):
        if self.err_px is None:
            return None
        dx = self.err_px[0] - self.aim_px_x
        dy = self.err_px[1] - self.aim_px_y
        return (dx * dx + dy * dy) ** 0.5

    def compute_desired_tilt(self):
        """Assumes state ALIGN/ALIGNED (the only states that servo)."""
        err_x = self.target_x - self.t_x
        err_y = self.target_y - self.t_y
        desired_roll = self.pid_x.update(-err_x, self.t_x_dot, self.dt)
        desired_pitch = self.pid_y.update(-err_y, self.t_y_dot, self.dt)
        return desired_roll, desired_pitch

    def compute_thrust(self):
        """Climb-rate command. Assumes ALIGN/ALIGNED."""
        e = self.lateral_error_px()
        centered = e is not None and e <= self.descend_xy_gate_px
        if self.mission_mode == 'perch':
            if centered:
                return min(self.perch_climb_thrust, self.thrust_max)
            return self.hover_thrust
        if centered:
            return max(self.land_descend_thrust, self.thrust_min)
        return self.hover_thrust

    def body_rates(self, desired_roll, desired_pitch, roll=0.0, pitch=0.0):
        """RATE mode only. Measured attitude defaults to level."""
        return (clamp(self.kp_att * (desired_roll - roll),
                      -self.max_body_rate, self.max_body_rate),
                clamp(self.kp_att * (desired_pitch - pitch),
                      -self.max_body_rate, self.max_body_rate))


def deg(rad):
    return rad * 180.0 / 3.141592653589793


def describe(sim, px, py, show_rate):
    roll, pitch = sim.compute_desired_tilt()
    e_px = sim.lateral_error_px()
    thrust = sim.compute_thrust()

    sat_r = abs(abs(roll) - sim.max_tilt) < 1e-9
    sat_p = abs(abs(pitch) - sim.max_tilt) < 1e-9

    print("")
    print("  detection      px=%.1f  py=%.1f      (centre %.0f, %.0f)"
          % (px, py, sim.half_w, sim.half_h))
    print("  image offset   dx=%+.1f px  dy=%+.1f px   %s / %s"
          % (sim.err_px[0], sim.err_px[1],
             "RIGHT of centre" if sim.err_px[0] > 0 else "LEFT of centre",
             "BELOW centre" if sim.err_px[1] > 0 else "ABOVE centre"))
    print("  normalized     t_x=%+.4f  t_y=%+.4f   (both / half_w=%.0f)"
          % (sim.t_x, sim.t_y, sim.half_w))
    print("  radial error   %.1f px    align_tol=%.0f  gate=%.0f  -> %s"
          % (e_px, sim.align_tolerance_px, sim.descend_xy_gate_px,
             "CENTRED" if e_px <= sim.descend_xy_gate_px else "off-centre"))
    print("")
    print("  ROLL   P%+.5f  I%+.5f  D%+.5f  ->  %+.5f rad = %+.3f deg%s"
          % (sim.pid_x.p_term, sim.pid_x.i_term, sim.pid_x.d_term,
             roll, deg(roll), "   [SATURATED]" if sat_r else ""))
    print("  PITCH  P%+.5f  I%+.5f  D%+.5f  ->  %+.5f rad = %+.3f deg%s"
          % (sim.pid_y.p_term, sim.pid_y.i_term, sim.pid_y.d_term,
             pitch, deg(pitch), "   [SATURATED]" if sat_p else ""))
    print("")
    print("  command        roll %+.3f deg -> lean %s"
          % (deg(roll), "RIGHT" if roll > 0 else "LEFT" if roll < 0 else "none"))
    print("                 pitch %+.3f deg -> lean %s"
          % (deg(pitch),
             "BACKWARD" if pitch > 0 else "FORWARD" if pitch < 0 else "none"))
    print("  thrust         %.3f  (%s)"
          % (thrust,
             "climb" if thrust > 0.5 else "descend" if thrust < 0.5 else "hold"))

    if show_rate:
        rr, pr = sim.body_rates(roll, pitch)
        print("  body rates     roll %+.4f rad/s (%+.2f deg/s)   "
              "pitch %+.4f rad/s (%+.2f deg/s)"
              % (rr, deg(rr), pr, deg(pr)))
    else:
        print("  body rates     0.0 / 0.0  (command_mode '%s': an ANGLE is "
              "sent, type_mask 7)" % sim.command_mode)


def sweep(p):
    """Polarity check: put the target off-centre and see which way we lean."""
    print("")
    print("POLARITY SWEEP -- mission_mode=%s  image_x_sign=%+.1f  "
          "image_y_sign=%+.1f" % (p.get('mission_mode'),
                                  float(p.get('image_x_sign', 1.0)),
                                  float(p.get('image_y_sign', 1.0))))
    print("P-term only (fresh PID each row, no I/D history).")
    print("")
    print("  %-28s %-22s %-16s %s"
          % ("target sits", "pixel", "roll / pitch (deg)", "vehicle leans"))
    print("  " + "-" * 84)

    cases = [
        ("RIGHT of centre",  IMAGE_WIDTH * 0.9, IMAGE_HEIGHT * 0.5),
        ("LEFT of centre",   IMAGE_WIDTH * 0.1, IMAGE_HEIGHT * 0.5),
        ("BELOW centre",     IMAGE_WIDTH * 0.5, IMAGE_HEIGHT * 0.9),
        ("ABOVE centre",     IMAGE_WIDTH * 0.5, IMAGE_HEIGHT * 0.1),
    ]
    for label, px, py in cases:
        sim = TiltSim(p)
        sim.target_callback(px, py)
        roll, pitch = sim.compute_desired_tilt()
        lean = []
        if abs(roll) > 1e-9:
            lean.append("RIGHT" if roll > 0 else "LEFT")
        if abs(pitch) > 1e-9:
            lean.append("BACKWARD" if pitch > 0 else "FORWARD")
        print("  %-28s (%4.0f,%4.0f)          %+7.3f / %+7.3f   %s"
              % (label, px, py, deg(roll), deg(pitch), " + ".join(lean) or "-"))

    print("")
    print("  Read it as: the lean must move the CAMERA so the target returns")
    print("  to the centre of the image. Work out which way the vehicle has")
    print("  to tip for YOUR mount, and check the last column agrees.")
    print("")


def main():
    args = sys.argv[1:]
    show_rate = '--rate' in args
    do_reset = '--reset' in args
    do_sweep = '--sweep' in args
    nums = [a for a in args if not a.startswith('--')]

    p = load_params()
    if do_sweep:
        sweep(p)
        return

    sim = TiltSim(p)

    print("")
    print("IBVS tilt simulator -- %s" % CONFIG)
    print("  mission_mode %s   command_mode %s   %dx%d"
          % (sim.mission_mode, sim.command_mode, sim.image_width, sim.image_height))
    print("  image_x_sign %+.1f   image_y_sign %+.1f   target (%.2f, %.2f)"
          % (sim.image_x_sign, sim.image_y_sign, sim.target_x, sim.target_y))
    print("  kp %.3f  ki %.3f  kd %.3f  i_max %.3f  max_tilt %.4f rad (%.2f deg)"
          % (sim.pid_x.kp, sim.pid_x.ki, sim.pid_x.kd, sim.pid_x.i_max,
             sim.max_tilt, deg(sim.max_tilt)))
    print("  control_rate %.1f Hz -> dt %.4f s" % (sim.control_rate, sim.dt))

    if len(nums) >= 2:
        sim.target_callback(float(nums[0]), float(nums[1]))
        describe(sim, float(nums[0]), float(nums[1]), show_rate)
        print("")
        return

    print("")
    print("Enter 'px py' per line ('r' reset, 'q' quit). PID state carries"
          " over%s." % (" -- DISABLED by --reset" if do_reset else ""))

    while True:
        try:
            line = raw_input("px py > ")          # noqa: F821  (py2 / ROS Noetic)
        except NameError:
            line = input("px py > ")
        except (EOFError, KeyboardInterrupt):
            print("")
            return

        line = line.strip()
        if not line:
            continue
        if line in ('q', 'quit', 'exit'):
            return
        if line in ('r', 'reset'):
            sim.reset()
            print("  PIDs reset.")
            continue

        parts = line.replace(',', ' ').split()
        if len(parts) < 2:
            print("  need two numbers: px py")
            continue
        try:
            px, py = float(parts[0]), float(parts[1])
        except ValueError:
            print("  need two numbers: px py")
            continue

        if do_reset:
            sim.reset()
        sim.target_callback(px, py)
        describe(sim, px, py, show_rate)


if __name__ == '__main__':
    main()
