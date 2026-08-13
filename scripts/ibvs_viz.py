#!/usr/bin/env python3
"""OpenCV direction visualizer for the IBVS pipeline (bench confirmation).

A black window with a single arrow from the center showing which way the
controller is commanding the UAV to move, decoded from the body-rate + thrust
setpoint on mavros/setpoint_raw/attitude. Up/down (thrust) is a separate
climb/descend indicator on the side, since a flat arrow can't show it.

Subscribe-only; it never sends anything to the FCU. Use it to confirm the
detection -> UDP -> receiver -> controller -> command path points the right way.

Body-frame sign map (FLU), matching ibvs_controller.compute_body_rates:
  body_rate.x (roll)  > 0  -> RIGHT     < 0 -> LEFT     (screen +x)
  body_rate.y (pitch) > 0  -> FORWARD   < 0 -> BACK     (screen up)
  thrust > 0.5 -> CLIMB    < 0.5 -> DESCEND    ~0.5 -> HOLD ALT

Run standalone (own window):
  rosrun ibvs_perching ibvs_viz.py _namespace:=red
Params: ~namespace, ~full_scale_rate (rad/s for a full-length arrow, def 0.35),
  ~win (window size px, def 720), ~out_png (also write each frame here),
  ~no_window (headless: skip cv2.imshow, only write out_png).
"""
import os
import math
import numpy as np
import rospy
from mavros_msgs.msg import AttitudeTarget
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    cv2 = None

BLACK = (0, 0, 0)
GREEN = (60, 230, 60)
RED = (60, 60, 235)
YELLOW = (40, 210, 235)
GREY = (120, 120, 120)
WHITE = (235, 235, 235)


class Viz(object):
    def __init__(self):
        ns = rospy.get_param('~namespace', os.environ.get('UAV_NAMESPACE', 'red'))
        self.ns = ns.strip('/')
        self.full_scale = float(rospy.get_param('~full_scale_rate', 0.35))  # rad/s
        self.eps = float(rospy.get_param('~rate_eps', 0.02))
        self.thr_eps = float(rospy.get_param('~thrust_eps', 0.02))
        self.timeout = float(rospy.get_param('~timeout', 1.0))
        self.S = int(rospy.get_param('~win', 720))
        self.out_png = rospy.get_param('~out_png', '')
        self.no_window = bool(rospy.get_param('~no_window', False))
        self.mode = rospy.get_param('~mission_mode', '?')

        self.sp = None            # (roll_rate, pitch_rate, thrust)
        self.sp_time = None
        self.state = '(waiting)'

        pre = '/%s/' % self.ns
        rospy.Subscriber(pre + 'mavros/setpoint_raw/attitude',
                         AttitudeTarget, self.on_sp, queue_size=1)
        rospy.Subscriber(pre + 'ibvs/state', String, self.on_state, queue_size=1)

    def on_sp(self, msg):
        self.sp = (msg.body_rate.x, msg.body_rate.y, msg.thrust)
        self.sp_time = rospy.Time.now()

    def on_state(self, msg):
        self.state = msg.data

    def fresh(self):
        return (self.sp_time is not None and
                (rospy.Time.now() - self.sp_time).to_sec() <= self.timeout)

    # ---- drawing -----------------------------------------------------------
    def draw(self):
        S = self.S
        img = np.zeros((S, S, 3), np.uint8)
        cx, cy = S // 2, S // 2
        R = int(S * 0.34)               # full-scale arrow radius

        # reference rings + crosshair
        cv2.circle(img, (cx, cy), R, (30, 30, 30), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), R // 2, (24, 24, 24), 1, cv2.LINE_AA)
        cv2.line(img, (cx - R - 20, cy), (cx + R + 20, cy), (22, 22, 22), 1)
        cv2.line(img, (cx, cy - R - 20), (cx, cy + R + 20), (22, 22, 22), 1)

        # compass labels
        cv2.putText(img, 'FWD', (cx - 22, cy - R - 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, GREY, 1, cv2.LINE_AA)
        cv2.putText(img, 'BACK', (cx - 28, cy + R + 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, GREY, 1, cv2.LINE_AA)
        cv2.putText(img, 'LEFT', (cx - R - 78, cy + 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, GREY, 1, cv2.LINE_AA)
        cv2.putText(img, 'RIGHT', (cx + R + 22, cy + 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, GREY, 1, cv2.LINE_AA)

        fresh = self.fresh()
        roll = pitch = 0.0
        thr = 0.5
        if fresh:
            roll, pitch, thr = self.sp

        # arrow vector: screen +x = RIGHT (roll>0), screen up = FWD (pitch>0)
        mag = math.hypot(roll, pitch)
        if fresh and mag > self.eps:
            scale = min(1.0, mag / self.full_scale)
            ux, uy = roll / mag, pitch / mag
            tipx = int(cx + ux * scale * R)
            tipy = int(cy - uy * scale * R)
            cv2.arrowedLine(img, (cx, cy), (tipx, tipy), GREEN, 10,
                            cv2.LINE_AA, tipLength=0.28)
            label = self.dir_label(roll, pitch)
        elif fresh:
            cv2.circle(img, (cx, cy), 14, YELLOW, 2, cv2.LINE_AA)
            label = 'HOLD'
        else:
            cv2.circle(img, (cx, cy), 10, GREY, 2, cv2.LINE_AA)
            label = 'NO COMMAND'
        cv2.circle(img, (cx, cy), 5, WHITE, -1, cv2.LINE_AA)

        # thrust / vertical indicator (right side bar)
        self.draw_thrust(img, thr, fresh)

        # header + footer text
        cv2.putText(img, 'IBVS  ns=%s  mode=%s' % (self.ns, self.mode),
                    (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 1, cv2.LINE_AA)
        col = GREEN if self.state in ('ALIGN', 'ALIGNED') else YELLOW
        cv2.putText(img, 'state: %s' % self.state, (20, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 1, cv2.LINE_AA)

        big = GREEN if (fresh and mag > self.eps) else (YELLOW if fresh else GREY)
        cv2.putText(img, label, (cx - self._tw(label, 1.1) // 2, S - 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, big, 2, cv2.LINE_AA)
        info = 'roll=%+.3f  pitch=%+.3f  thrust=%.3f' % (roll, pitch, thr)
        cv2.putText(img, info, (cx - self._tw(info, 0.6) // 2, S - 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv2.LINE_AA)
        cv2.putText(img, 'subscribe-only  -  never commands the FCU', (20, S - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREY, 1, cv2.LINE_AA)
        return img

    def draw_thrust(self, img, thr, fresh):
        S = self.S
        x = S - 70
        top, bot = 120, S - 140
        cv2.rectangle(img, (x - 22, top), (x + 22, bot), (35, 35, 35), 1)
        midy = (top + bot) // 2
        cv2.line(img, (x - 22, midy), (x + 22, midy), (60, 60, 60), 1)  # 0.5 hover
        # fill proportional to thrust (0..1), from mid
        t = max(0.0, min(1.0, thr))
        y = int(bot - t * (bot - top))
        up = fresh and thr > 0.5 + self.thr_eps
        down = fresh and thr < 0.5 - self.thr_eps
        col = GREEN if up else (RED if down else GREY)
        cv2.rectangle(img, (x - 20, min(y, midy)), (x + 20, max(y, midy)), col, -1)
        word = 'CLIMB' if up else ('DESCEND' if down else 'HOLD')
        cv2.putText(img, word, (x - 44, top - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, col, 1, cv2.LINE_AA)
        cv2.putText(img, 'UP', (x - 10, top - 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, GREY, 1, cv2.LINE_AA)
        cv2.putText(img, 'DOWN', (x - 24, bot + 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, GREY, 1, cv2.LINE_AA)

    def dir_label(self, roll, pitch):
        parts = []
        if pitch > self.eps:
            parts.append('FORWARD')
        elif pitch < -self.eps:
            parts.append('BACK')
        if roll > self.eps:
            parts.append('RIGHT')
        elif roll < -self.eps:
            parts.append('LEFT')
        return ' + '.join(parts) if parts else 'HOLD'

    def _tw(self, text, scale):
        (w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        return w

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            img = self.draw()
            if self.out_png:
                cv2.imwrite(self.out_png, img)
            if not self.no_window:
                cv2.imshow('IBVS direction', img)
                if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):  # ESC/q
                    rospy.signal_shutdown('user quit')
                    break
            rate.sleep()
        if not self.no_window:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    if cv2 is None:
        raise SystemExit("ibvs_viz: OpenCV (cv2) not available -- "
                         "install python3-opencv")
    rospy.init_node('ibvs_viz')
    Viz().spin()
