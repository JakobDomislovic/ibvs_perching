#!/usr/bin/env python

"""
Keyboard "RC transmitter" for SIMULATION.

Stands in for the real radio when testing the real-world engagement flow
in SITL: fly the vehicle manually from the keyboard, let the vision module
(or a manual `rostopic pub`) publish ibvs/target_point, and watch the
controller engage exactly like it will on the field. Works through
mavros/rc/override, so ArduPilot sees real RC input and reports it back on
mavros/rc/in -- the rc_to_joy.py bridge works in simulation unchanged.

Run it in its own terminal pane (it grabs the keyboard):

    rosrun ibvs_perching keyboard_rc.py __ns:=$UAV_NAMESPACE

Keys (sticks spring back to center on release):
    arrows      left/right = roll, up/down = pitch (forward/back)
    w / s       climb / descend (throttle)
    a / d       yaw left / right
    space       center all sticks
    i / k       call ibvs/start / ibvs/stop -- the "IBVS button" that a
                real joystick will have; with engage_needs_start this is
                what lets the next tag detection take over
    1 / 2 / 3   mode STABILIZE / ALT_HOLD / LOITER   <- "pilot takes over"
    g           mode GUIDED_NOGPS                    <- "pilot hands back"
    o / p       arm / disarm (disarm mid-air = crash; it is a simulator)
    h           help    q  quit (releases all overrides)

Fly in ALT_HOLD (2): centered throttle holds altitude, w/s command
climb/descent -- much easier on a keyboard than STABILIZE. Typical test:
    o (arm) -> 2 (ALT_HOLD) -> hold w (climb) -> arrows over the tag ->
    i (start IBVS) -> next detection engages GUIDED_NOGPS by itself
    -> 2 (take over, one-shot: controller must NOT steal the mode back)
    -> g (hand control back), or i again to re-arm the engagement

While disarmed the throttle idles LOW (arming with mid throttle is
rejected by ArduPilot); once armed it springs to center (1500).

Parameters:
    ~channel/{throttle,roll,pitch,yaw}  RC channel indices (0-based, same
                                        convention as rc_to_joy.yaml;
                                        defaults match RCMAP 1..4)
    ~step_tilt      roll/pitch/yaw deflection [pwm] (default 150)
    ~step_throttle  throttle deflection [pwm] (default 250)
    ~hold_time      seconds a keypress keeps deflecting; keyboard
                    autorepeat refreshes it while held (default 0.6)
    ~rate           publish rate [Hz] (default 20)
"""

import os
import select
import sys
import termios
import time
import tty

import rospy
from mavros_msgs.msg import OverrideRCIn, ParamValue, State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from std_srvs.srv import Trigger

PWM_MID = 1500
PWM_LOW = 1100
CHAN_RELEASE = OverrideRCIn.CHAN_RELEASE      # 0: give the channel back
CHAN_NOCHANGE = OverrideRCIn.CHAN_NOCHANGE    # 65535: leave untouched

HELP = ("arrows roll/pitch | w/s climb/descend | a/d yaw | space center | "
        "i START IBVS  k stop | 1 STAB 2 ALT_HOLD 3 LOITER g GUIDED_NOGPS | "
        "o arm p disarm | q quit")


class KeyboardRc:

    def __init__(self):
        self.ch_throttle = rospy.get_param('~channel/throttle', 2)
        self.ch_roll = rospy.get_param('~channel/roll', 0)
        self.ch_pitch = rospy.get_param('~channel/pitch', 1)
        self.ch_yaw = rospy.get_param('~channel/yaw', 3)
        self.step_tilt = rospy.get_param('~step_tilt', 150)
        self.step_throttle = rospy.get_param('~step_throttle', 250)
        self.hold_time = rospy.get_param('~hold_time', 0.6)
        self.rate = rospy.get_param('~rate', 20.0)

        self.armed = False
        self.mode = ''
        # per-axis deflection [-1..1] and the time of the last keypress
        self.deflection = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.0, 'yaw': 0.0}
        self.pressed_at = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.0, 'yaw': 0.0}

        self.override_pub = rospy.Publisher(
            'mavros/rc/override', OverrideRCIn, queue_size=1)
        rospy.Subscriber('mavros/state', State, self.state_callback, queue_size=1)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        # 'i'/'k' act as the IBVS button of the future real joystick
        self.ibvs_start_srv = rospy.ServiceProxy('ibvs/start', Trigger)
        self.ibvs_stop_srv = rospy.ServiceProxy('ibvs/stop', Trigger)

    def state_callback(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def ensure_override_accepted(self):
        """Make ArduPilot accept our overrides: SYSID_MYGCS must be 1.

        ArduPilot only honors RC_CHANNELS_OVERRIDE coming from the system
        id stored in SYSID_MYGCS, which defaults to 255 (MAVProxy) --
        mavros sends with system id 1, so without this every override is
        SILENTLY ignored and the keyboard "does nothing". Retries while
        SITL/mavros are still coming up.
        """
        param_set = rospy.ServiceProxy('mavros/param/set', ParamSet)
        deadline = time.time() + 60.0
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                res = param_set(param_id='SYSID_MYGCS',
                                value=ParamValue(integer=1, real=0.0))
                if res.success:
                    print('keyboard_rc: SYSID_MYGCS=1 -- RC override enabled')
                    return
            except (rospy.ServiceException, rospy.ROSException):
                pass
            print('keyboard_rc: waiting for mavros/FCU (setting SYSID_MYGCS=1)...')
            time.sleep(2.0)
        print('keyboard_rc: WARNING: could not set SYSID_MYGCS=1 -- '
              'ArduPilot will IGNORE the overrides (sticks will do nothing)')

    def press(self, axis, value):
        self.deflection[axis] = value
        self.pressed_at[axis] = time.time()

    def set_mode(self, mode):
        try:
            self.set_mode_srv(base_mode=0, custom_mode=mode)
        except rospy.ServiceException as exc:
            self.status_line('set_mode failed: %s' % exc)

    def arm(self, value):
        try:
            self.arming_srv(value)
        except rospy.ServiceException as exc:
            self.status_line('arming failed: %s' % exc)

    def call_ibvs(self, srv, name):
        try:
            res = srv()
            self.status_line('%s: %s' % (name, res.message))
        except (rospy.ServiceException, rospy.ROSException) as exc:
            self.status_line('%s failed (is the ibvs node running?): %s'
                             % (name, exc))

    def handle_key(self, key):
        if key == '\x1b[D':
            self.press('roll', -1.0)
        elif key == '\x1b[C':
            self.press('roll', 1.0)
        elif key == '\x1b[A':
            self.press('pitch', 1.0)      # forward
        elif key == '\x1b[B':
            self.press('pitch', -1.0)     # back
        elif key == 'w':
            self.press('throttle', 1.0)
        elif key == 's':
            self.press('throttle', -1.0)
        elif key == 'a':
            self.press('yaw', -1.0)
        elif key == 'd':
            self.press('yaw', 1.0)
        elif key == ' ':
            for axis in self.deflection:
                self.deflection[axis] = 0.0
        elif key == '1':
            self.set_mode(State.MODE_APM_COPTER_STABILIZE)
        elif key == '2':
            self.set_mode(State.MODE_APM_COPTER_ALT_HOLD)
        elif key == '3':
            self.set_mode(State.MODE_APM_COPTER_LOITER)
        elif key == 'g':
            self.set_mode(State.MODE_APM_COPTER_GUIDED_NOGPS)
        elif key == 'i':
            self.call_ibvs(self.ibvs_start_srv, 'ibvs/start')
        elif key == 'k':
            self.call_ibvs(self.ibvs_stop_srv, 'ibvs/stop')
        elif key == 'o':
            self.arm(True)
        elif key == 'p':
            self.arm(False)
        elif key == 'h':
            print('\r\n' + HELP + '\r')

    def spring_back(self):
        """Sticks recenter when the key (autorepeat) stops refreshing them."""
        now = time.time()
        for axis in self.deflection:
            if now - self.pressed_at[axis] > self.hold_time:
                self.deflection[axis] = 0.0

    def build_override(self):
        # ArduPilot convention: roll/yaw right and throttle up = higher pwm,
        # pitch stick FORWARD = lower pwm (RC2 is reversed on purpose)
        # throttle idles LOW while disarmed so GCS arming is not rejected
        throttle_center = PWM_MID if self.armed else PWM_LOW
        pwm = {
            self.ch_roll: PWM_MID + int(self.step_tilt * self.deflection['roll']),
            self.ch_pitch: PWM_MID - int(self.step_tilt * self.deflection['pitch']),
            self.ch_yaw: PWM_MID + int(self.step_tilt * self.deflection['yaw']),
            self.ch_throttle: throttle_center
            + int(self.step_throttle * self.deflection['throttle']),
        }
        msg = OverrideRCIn()
        msg.channels = [CHAN_NOCHANGE] * len(msg.channels)
        for index, value in pwm.items():
            msg.channels[index] = value
        return msg, pwm

    def status_line(self, extra=''):
        pwm = self.build_override()[1]
        sys.stdout.write(
            '\r%-12s %s  R %4d  P %4d  T %4d  Y %4d   %s\x1b[K' % (
                self.mode or '?', 'ARMED' if self.armed else 'disarmed',
                pwm[self.ch_roll], pwm[self.ch_pitch],
                pwm[self.ch_throttle], pwm[self.ch_yaw], extra))
        sys.stdout.flush()

    def release_all(self):
        msg = OverrideRCIn()
        msg.channels = [CHAN_RELEASE] * len(msg.channels)
        for _ in range(3):
            self.override_pub.publish(msg)
            rospy.sleep(0.05)

    def read_keys(self, timeout):
        """Read pending keypresses, decoding arrow-key escape sequences.

        Must read the RAW fd (os.read): sys.stdin.read(1) buffers the rest
        of an escape sequence inside Python where select() cannot see it,
        which splits arrows into a lone ESC -- roll/pitch keys would be
        silently dropped while plain letter keys keep working.
        """
        keys = []
        if not select.select([sys.stdin], [], [], timeout)[0]:
            return keys
        data = os.read(sys.stdin.fileno(), 64).decode(errors='ignore')
        # drain whatever arrived together (escape sequences, autorepeat)
        while select.select([sys.stdin], [], [], 0)[0]:
            data += os.read(sys.stdin.fileno(), 64).decode(errors='ignore')
        i = 0
        while i < len(data):
            # arrows arrive as ESC [ X (CSI) or ESC O X (application mode,
            # e.g. under some tmux/terminal configs); normalize to ESC [ X
            if data[i] == '\x1b' and data[i + 1:i + 2] in ('[', 'O'):
                keys.append('\x1b[' + data[i + 2:i + 3])
                i += 3
            else:
                keys.append(data[i])
                i += 1
        return keys

    def run(self):
        print(HELP + '\r')
        period = 1.0 / self.rate
        while not rospy.is_shutdown():
            for key in self.read_keys(period):
                if key == 'q':
                    return
                self.handle_key(key)
            self.spring_back()
            self.override_pub.publish(self.build_override()[0])
            self.status_line()


def main():
    rospy.init_node('keyboard_rc')
    node = KeyboardRc()
    node.ensure_override_accepted()
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        node.run()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        node.release_all()
        print('\nkeyboard_rc: overrides released')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
