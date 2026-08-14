#!/usr/bin/env python

"""
Image-Based Visual Servoing controller.

Publishes DIRECTLY to mavros/setpoint_raw/attitude (body rates + thrust),
bypassing the uav_ros_stack tracker/controller entirely. Arming and mode
switching (GUIDED_NOGPS) are done by the ibvs/takeoff service -- this node
only ever streams AttitudeTarget setpoints.

VISION MODULE INTERFACE (topic `ibvs/target_point`, geometry_msgs/PointStamped):
    The controller is agnostic to WHAT is being tracked and needs NO camera
    calibration. Any vision module (ArUco today, a plain object detector
    tomorrow) publishes the PIXEL coordinates of the point it wants
    centered in the camera image:
        point.x  pixel column (u), 0 .. image_width
        point.y  pixel row (v), 0 .. image_height
        point.z  unused, always 0.0 -- there is no depth (see below)
    Publishing on this topic at all means "target in sight": the state
    machine shows TAG_IN_SIGHT and ibvs/start will engage. ~image_width /
    ~image_height must be set (once, in the launch file) to the SAME
    resolution the vision module is using, so the controller can find the
    frame center.

    The camera is assumed rigidly mounted looking straight down with image
    RIGHT = body FORWARD (the kopterworx down_facing_camera mount): a point
    seen right-of-center in the image lies to the right of the vehicle.

    NO DEPTH: a bare pixel center (no marker size, no calibrated
    intrinsics) carries no distance information, so the controller cannot
    regulate a metric standoff above the target. FOR NOW it does not
    descend at all: ALIGN/ALIGNED only center the target laterally and
    hold the odometry height the vehicle was at when servoing started
    -- see compute_thrust and the README limitations.

IMPORTANT -- how ArduPilot interprets the "thrust" field:
    In GUIDED / GUIDED_NOGPS mode ArduPilot treats AttitudeTarget.thrust
    as a CLIMB RATE command, not raw motor thrust (unless GUID_OPTIONS
    bit 3 is set, which we assume it is not):
        0.0 -> descend at maximum rate (PILOT_SPEED_DN)
        0.5 -> zero climb rate (hold altitude / hover)
        1.0 -> climb at maximum rate (PILOT_SPEED_UP / WPNAV_SPEED_UP)
    This is exactly why hover_thrust defaults to 0.5, and why the vehicle
    only takes off when we command thrust ABOVE 0.5 (climb_thrust).

Control split:
  - Z:   FOR NOW, no descent -- there is no depth to regulate against, and
         no open-loop descent strategy is enabled yet either. CLIMB sends
         climb_thrust (the takeoff); WAIT_ARM/HOVER/TAG_IN_SIGHT send a
         plain hover_thrust. ALIGN/ALIGNED hold the odometry height
         latched when servoing started (hold_height, see transition()),
         regulated the same way as the X-Y position hold below (P term +
         velocity damping) rather than trusting hover_thrust alone to
         hold still open-loop. Centering the target laterally does not
         bring the vehicle any closer to it.
  - X-Y: closed-loop IBVS as a CASCADE. A raw body-rate command
         proportional to position error is unstable (the rate integrates
         into an unbounded tilt -- flight tested, it flips the vehicle),
         so instead:
             desired_tilt = PID_xy(target error)   (clamped small)
             body_rate    = kp_att * (desired_tilt - current_tilt)
         The error is the target's pixel offset from the frame center,
         expressed as a FRACTION of the half-frame (independent of
         image_width/image_height, so gains don't need retuning if the
         camera resolution changes). Attitude and body velocity are taken
         from odometry. All in the body FLU convention (mavros converts
         FLU->FRD for MAVLink). Sign conventions (FLU, ROS euler): +pitch =
         nose down = +x accel, +roll = right side down = -y accel.

         The PID derivative term uses body velocity as a damping proxy.
         With a metric error this was EXACT (d(err)/dt = -vel for a
         stationary tag); with a frame-fraction error it is only an
         approximation, since the same velocity produces a bigger apparent
         pixel motion the closer the vehicle is to the target -- a scaling
         that came for free with a real depth measurement and cannot be
         corrected without one.

State machine (this is what makes the controller "modal"):

    WAIT_ARM --(ibvs/takeoff called; armed & GUIDED_NOGPS confirmed)--> CLIMB
    WAIT_ARM --(engage_on_target; armed; point received)--> ALIGN / TAG_LOST
    CLIMB --(takeoff_height reached, servoing NOT started)--> HOVER / TAG_IN_SIGHT
    CLIMB --(takeoff_height reached, started & tag seen)--> ALIGN
    CLIMB --(takeoff_height reached, started, NO tag)--> TAG_LOST
    HOVER <--(tag detection appears / disappears)--> TAG_IN_SIGHT
    HOVER/TAG_IN_SIGHT --(ibvs/start called & tag seen)--> ALIGN
    ALIGN --(|error| < tol for align_dwell_time)--> ALIGNED
    ALIGNED --(|error| > tol * hysteresis)--> ALIGN
    (any state) --(disarmed / mode changed)--> WAIT_ARM
    (any flying state) --(ibvs/stop called)--> HOVER
    (ALIGN/ALIGNED) --(no tag for tag_timeout)--> TAG_LOST
    TAG_LOST --(tag seen again)--> ALIGN

TAG_IN_SIGHT behaves exactly like HOVER (position hold at the same latched
point); it is a status distinction for the operator: the detector currently
sees the tag, so `ibvs/start` will engage immediately. Call ibvs/start when
`ibvs/state` shows TAG_IN_SIGHT.

Two-step mission (both std_srvs/Trigger):
    1. `ibvs/takeoff` -- switches to GUIDED_NOGPS, arms, climbs to
       takeoff_height meters (climb_settle_time is the fallback timeout),
       then HOLDS position (HOVER).
    2. `ibvs/start`   -- starts servoing toward the tag (ALIGN).
    `ibvs/stop` aborts servoing back to HOVER at any time. Arming the
    vehicle manually does NOT make it climb; only ibvs/takeoff does.
    Set ~auto_start true to skip the ibvs/start gate (takeoff flows
    straight into ALIGN).

REAL WORLD (~engage_on_target: true, see startup/real_world):
    The safety pilot flies the vehicle manually (e.g. STABILIZE) and a
    fresh `ibvs/target_point` engages the controller -- no takeoff
    service and no position_hold-style handshake. With
    ~engage_needs_start: true (the shipped configs) engagement is
    BUTTON-GATED: the pilot calls `ibvs/start` first ('i' on the sim
    keyboard joystick, a joystick button on the real RC) and the next
    point is what engages; with it false the very first point engages.
    On engagement (while armed) the controller switches the FCU to
    GUIDED_NOGPS itself and, once mavros/state confirms armed +
    GUIDED_NOGPS, goes straight to ALIGN (the vehicle is already
    airborne, CLIMB is skipped).
    The software mode switch is ONE-SHOT: if the safety pilot takes back
    control with the RC mode switch, the controller never grabs the mode
    again on its own -- flipping the RC switch back to GUIDED_NOGPS
    re-engages (servoing stays enabled), and `ibvs/start` re-arms the
    one-shot so the next target point can engage again. `ibvs/stop`
    disables servoing as usual.

Thrust ("climb rate") per state:
    WAIT_ARM  hover_thrust (neutral; ignored anyway while disarmed)
    CLIMB     climb_thrust (constant climb -> this IS the takeoff)
    HOVER     hover_thrust (hold altitude, wait for ibvs/start)
    ALIGN     regulated toward hold_height (centering only, no descent for now)
    ALIGNED   regulated toward hold_height (centering only, no descent for now)
    TAG_LOST  hover_thrust (hold altitude, wait for re-detection)
"""

import math

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


WAIT_ARM = 'WAIT_ARM'
CLIMB = 'CLIMB'
HOVER = 'HOVER'
TAG_IN_SIGHT = 'TAG_IN_SIGHT'
ALIGN = 'ALIGN'
ALIGNED = 'ALIGNED'
TAG_LOST = 'TAG_LOST'

# states that hold the latched position (everything flying except servoing)
HOLD_STATES = (CLIMB, HOVER, TAG_IN_SIGHT, TAG_LOST)


def clamp(value, low, high):
    return max(low, min(high, value))


class Pid:
    """Standard PID with output clamp and integral anti-windup.

    The derivative term takes error_dot directly (we feed it -body_velocity,
    see module docstring) rather than differentiating the error signal.
    """

    def __init__(self, kp, ki, kd, out_min, out_max, i_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_max = i_max      # clamp on the INTEGRAL CONTRIBUTION (ki * integral)
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, error_dot, dt):
        i_term = 0.0
        if self.ki > 0.0:
            self.integral += error * dt
            # anti-windup: keep the integral contribution bounded
            self.integral = clamp(self.integral,
                                  -self.i_max / self.ki, self.i_max / self.ki)
            i_term = self.ki * self.integral

        out = self.kp * error + i_term + self.kd * error_dot
        return clamp(out, self.out_min, self.out_max)


class IbvsController:

    def __init__(self):
        self.control_rate = rospy.get_param('~control_rate', 20.0)
        self.dt = 1.0 / self.control_rate

        # Camera resolution -- the ONLY thing the controller needs to know
        # about the camera (no intrinsics, no calibration). Must match the
        # vision module publishing ibvs/target_point (set once, together,
        # in the launch file).
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)

        # Z axis ("climb rate" through the thrust field). No depth is
        # available, so there is no standoff to regulate toward, and FOR
        # NOW no descent strategy is enabled either -- ALIGN/ALIGNED just
        # hold hover_thrust (see compute_thrust).
        self.hover_thrust = rospy.get_param('~hover_thrust', 0.5)
        self.climb_thrust = rospy.get_param('~climb_thrust', 0.6)
        self.thrust_min = rospy.get_param('~thrust_min', 0.35)
        self.thrust_max = rospy.get_param('~thrust_max', 0.7)
        self.takeoff_height = rospy.get_param('~takeoff_height', 2.0)
        # Altitude hold during ALIGN/ALIGNED, regulated toward hold_height
        # (latched at the ODOMETRY height when servoing starts -- see
        # transition() -- NOT takeoff_height, which CLIMB may over/undershoot,
        # and not simply an open-loop hover_thrust, which can drift over an
        # arbitrarily long wait in HOVER before ibvs/start is called). Same
        # P+damping structure as the X-Y position hold below.
        self.kp_alt_hold = rospy.get_param('~kp_alt_hold', 0.3)
        self.kv_alt_hold = rospy.get_param('~kv_alt_hold', 0.2)

        # X-Y axis (IBVS cascade: PID on tag error -> desired tilt -> body rate)
        self.target_x = rospy.get_param('~target_x', 0.0)
        self.target_y = rospy.get_param('~target_y', 0.0)
        self.max_tilt = rospy.get_param('~max_tilt', 0.15)
        self.kp_att = rospy.get_param('~kp_att', 1.5)
        self.max_body_rate = rospy.get_param('~max_body_rate', 0.35)
        # Position hold outside ALIGN/ALIGNED. Level attitude alone does NOT
        # hover in place: attitude trim bias (AHRS_TRIM in the FCU params)
        # gives a constant lateral push, and velocity braking only limits the
        # resulting drift to a terminal speed (flight-tested ~0.1 m/s,
        # forever). Holding the latched position cancels the bias.
        self.kp_hover = rospy.get_param('~kp_hover', 0.15)
        self.kv_hover = rospy.get_param('~kv_hover', 0.25)

        # PID gains (I and D default to 0 -- pure P until tuned otherwise)
        kp_xy = rospy.get_param('~pid_xy/kp', 0.15)
        ki_xy = rospy.get_param('~pid_xy/ki', 0.0)
        kd_xy = rospy.get_param('~pid_xy/kd', 0.0)
        i_max_xy = rospy.get_param('~pid_xy/i_max', 0.05)
        self.pid_x = Pid(kp_xy, ki_xy, kd_xy, -self.max_tilt, self.max_tilt, i_max_xy)
        self.pid_y = Pid(kp_xy, ki_xy, kd_xy, -self.max_tilt, self.max_tilt, i_max_xy)

        # State machine timing / thresholds
        self.climb_settle_time = rospy.get_param('~climb_settle_time', 3.0)
        # Alignment check is done in PIXELS (unlike the frame-fraction error
        # that drives the PID cascade above) -- easy to reason about against
        # a live video feed ("must be within 20 px of center"), and exact
        # regardless of image aspect ratio (see compute_body_rates).
        self.align_tolerance_px = rospy.get_param('~align_tolerance_px', 20.0)
        self.align_dwell_time = rospy.get_param('~align_dwell_time', 2.0)
        self.align_hysteresis = rospy.get_param('~align_hysteresis', 1.5)
        self.tag_timeout = rospy.get_param('~tag_timeout', 1.0)

        # Servoing gate: takeoff only climbs and hovers; flying to the tag
        # starts when the ibvs/start service is called (or ~auto_start: true).
        self.servo_active = rospy.get_param('~auto_start', False)
        # Takeoff gate: being armed in GUIDED_NOGPS alone does NOT climb;
        # the climb happens only after the ibvs/takeoff service is called.
        self.takeoff_requested = False
        # Real-world engagement gate: the vehicle is flown MANUALLY (e.g.
        # STABILIZE) and publishing on ibvs/target_point IS the "go
        # autonomous" signal (replaces the position_hold handshake of the
        # uav_ros_stack). On the first fresh point while armed the
        # controller switches the FCU to GUIDED_NOGPS itself and goes
        # straight to ALIGN -- no takeoff/climb, it is already airborne.
        # engage_armed makes the software mode switch ONE-SHOT: after a
        # safety-pilot RC takeover the controller never steals the mode
        # back just because the vision module keeps publishing; call
        # ibvs/start to re-arm it.
        self.engage_on_target = rospy.get_param('~engage_on_target', False)
        # Button-gated engagement: when true, the controller does NOT
        # engage on the very first point -- somebody must call ibvs/start
        # first ('i' on the sim keyboard joystick, a joystick button on
        # the real RC). The point that arrives after the button press is
        # what engages.
        self.engage_needs_start = rospy.get_param('~engage_needs_start', False)
        self.engage_armed = self.engage_on_target and not self.engage_needs_start

        self.state = WAIT_ARM
        self.state_entered_at = rospy.Time.now()
        self.aligned_since = None

        self.armed = False
        self.mode = ''
        # target error reconstructed from the vision module's pixel point,
        # as a fraction of the half-frame: t_x forward, t_y left (0 = the
        # target is exactly centered). No depth -- see module docstring.
        self.t_x = None
        self.t_y = None
        self.last_tag_time = None
        self.last_odom = None
        # (x, y) in the local frame that HOVER/TAG_LOST hold on to
        self.hold_position = None
        # odometry z that ALIGN/ALIGNED hold on to (latched on entering
        # ALIGN -- see transition() and compute_thrust)
        self.hold_height = None

        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.state_pub = rospy.Publisher('ibvs/state', String, queue_size=1, latch=True)
        # latch the initial state too -- transitions alone would leave the
        # topic silent until the first state change
        self.state_pub.publish(String(data=self.state))

        rospy.Subscriber('mavros/state', State, self.mavros_state_callback, queue_size=1)
        rospy.Subscriber('ibvs/target_point', PointStamped, self.target_callback, queue_size=1)
        rospy.Subscriber('mavros/local_position/odom', Odometry,
                         self.odom_callback, queue_size=1)

        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        rospy.Service('ibvs/takeoff', Trigger, self.handle_takeoff)
        rospy.Service('ibvs/start', Trigger, self.handle_start)
        rospy.Service('ibvs/stop', Trigger, self.handle_stop)

        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

    def handle_takeoff(self, _req):
        """Full takeoff sequence: GUIDED_NOGPS -> arm -> CLIMB -> HOVER."""
        if self.state != WAIT_ARM:
            return TriggerResponse(
                success=False,
                message="already flying (state %s)" % self.state)

        # Allow climbing as soon as armed+mode are confirmed by mavros/state.
        self.takeoff_requested = True
        try:
            if self.mode != State.MODE_APM_COPTER_GUIDED_NOGPS:
                mode_res = self.set_mode_srv(
                    base_mode=0, custom_mode=State.MODE_APM_COPTER_GUIDED_NOGPS)
                if not mode_res.mode_sent:
                    self.takeoff_requested = False
                    return TriggerResponse(success=False,
                                           message="set_mode GUIDED_NOGPS rejected")
                rospy.sleep(2.0)

            if not self.armed:
                arm_res = self.arming_srv(True)
                if not arm_res.success:
                    self.takeoff_requested = False
                    return TriggerResponse(success=False,
                                           message="arming rejected (result %d)" % arm_res.result)
        except rospy.ServiceException as exc:
            self.takeoff_requested = False
            return TriggerResponse(success=False, message="mavros service error: %s" % exc)

        rospy.loginfo("ibvs_controller: TAKEOFF accepted (ibvs/takeoff)")
        return TriggerResponse(
            success=True,
            message="taking off: climbing %.1fs then holding; call ibvs/start to align"
                    % self.climb_settle_time)

    def handle_start(self, _req):
        self.servo_active = True
        # re-arm the one-shot mid-flight engagement (real world): the next
        # fresh target point may switch the FCU to GUIDED_NOGPS again
        self.engage_armed = self.engage_on_target
        rospy.loginfo("ibvs_controller: servoing STARTED (ibvs/start)")
        return TriggerResponse(success=True, message="IBVS servoing started")

    def handle_stop(self, _req):
        self.servo_active = False
        rospy.loginfo("ibvs_controller: servoing STOPPED (ibvs/stop)")
        return TriggerResponse(success=True, message="IBVS servoing stopped, holding position")

    def mavros_state_callback(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def target_callback(self, msg):
        """Vision-module pixel point -> lateral error, as a fraction of the
        half-frame (independent of image_width/image_height, so gains and
        tolerances don't need retuning if the camera resolution changes).
        Down camera with image right = body forward.
        """
        norm_x = (msg.point.x - self.image_width / 2.0) / (self.image_width / 2.0)
        norm_y = (msg.point.y - self.image_height / 2.0) / (self.image_height / 2.0)
        self.t_x = norm_x
        self.t_y = -norm_y
        self.last_tag_time = rospy.Time.now()

        # real world: this point is the "go autonomous" signal
        if self.engage_armed and self.armed and self.state == WAIT_ARM:
            self.request_engage()

    def request_engage(self):
        """Seize control mid-flight (real world): GUIDED_NOGPS + servoing.

        Called on the first fresh target point while the vehicle is flown
        manually. One-shot: engage_armed stays False afterwards, so after a
        safety-pilot RC takeover the controller cannot grab the mode back
        on its own -- the pilot re-engages with the RC mode switch, or
        ibvs/start re-arms this. The actual ALIGN transition happens in the
        state machine only once mavros/state confirms armed + GUIDED_NOGPS.
        """
        self.engage_armed = False
        self.servo_active = True
        rospy.loginfo("ibvs_controller: target point received while flying "
                      "manually -> engaging (GUIDED_NOGPS)")
        if self.mode == State.MODE_APM_COPTER_GUIDED_NOGPS:
            return
        try:
            mode_res = self.set_mode_srv(
                base_mode=0, custom_mode=State.MODE_APM_COPTER_GUIDED_NOGPS)
            if not mode_res.mode_sent:
                rospy.logerr("ibvs_controller: engage failed -- set_mode "
                             "GUIDED_NOGPS rejected; call ibvs/start to re-arm")
        except rospy.ServiceException as exc:
            rospy.logerr("ibvs_controller: engage failed -- mavros service "
                         "error: %s; call ibvs/start to re-arm", exc)

    def odom_callback(self, msg):
        self.last_odom = msg

    def latch_hold_position(self):
        if self.last_odom is not None:
            p = self.last_odom.pose.pose.position
            self.hold_position = (p.x, p.y)
        else:
            self.hold_position = None

    def transition(self, new_state):
        if new_state != self.state:
            rospy.loginfo("ibvs_controller: %s -> %s", self.state, new_state)
            # entering closed-loop servoing from a non-servoing state:
            # start the PIDs fresh (drops any stale integral), and latch
            # the CURRENT odometry height as the Z hold target -- NOT
            # takeoff_height (see compute_thrust)
            if new_state == ALIGN and self.state not in (ALIGN, ALIGNED):
                self.pid_x.reset()
                self.pid_y.reset()
                if self.last_odom is not None:
                    self.hold_height = self.last_odom.pose.pose.position.z
            # Latch the spot the hold states keep: the takeoff point when
            # CLIMB starts, or wherever the vehicle is when servoing stops /
            # the tag is lost. HOVER <-> TAG_IN_SIGHT keep the same latch
            # (only the label changes), as does CLIMB -> HOVER/TAG_IN_SIGHT.
            if new_state == CLIMB or new_state == TAG_LOST or \
                    (new_state == HOVER and self.state not in (CLIMB, TAG_IN_SIGHT)):
                self.latch_hold_position()
            self.state = new_state
            self.state_entered_at = rospy.Time.now()
            if new_state != ALIGN:
                self.aligned_since = None
            self.state_pub.publish(String(data=new_state))

    def time_in_state(self):
        return (rospy.Time.now() - self.state_entered_at).to_sec()

    def tag_is_fresh(self):
        if self.last_tag_time is None:
            return False
        return (rospy.Time.now() - self.last_tag_time).to_sec() <= self.tag_timeout

    def update_state_machine(self):
        ready_to_fly = self.armed and self.mode == State.MODE_APM_COPTER_GUIDED_NOGPS

        # Global safety transition: falling out of armed+GUIDED_NOGPS always
        # drops back to WAIT_ARM, regardless of current state.
        if not ready_to_fly:
            self.transition(WAIT_ARM)
        elif self.state == WAIT_ARM:
            # Armed + GUIDED_NOGPS alone is not enough: climb only when the
            # ibvs/takeoff service asked for it.
            if self.takeoff_requested:
                self.transition(CLIMB)
            # Mid-flight engagement (real world): the vehicle is already
            # airborne, skip CLIMB and servo right away. Reached either by
            # the software mode switch after a target point (request_engage)
            # or by the safety pilot flipping the RC switch to GUIDED_NOGPS
            # while servoing is enabled.
            elif self.engage_on_target and self.servo_active:
                self.transition(ALIGN if self.tag_is_fresh() else TAG_LOST)
        elif self.state == CLIMB:
            # climb until takeoff_height; climb_settle_time is the fallback
            # timeout in case odometry never reports the altitude
            reached_height = (
                self.last_odom is not None and
                self.last_odom.pose.pose.position.z >= self.takeoff_height)
            if reached_height or self.time_in_state() >= self.climb_settle_time:
                self.takeoff_requested = False   # consumed; next takeoff needs a new call
                if not self.servo_active:
                    self.transition(TAG_IN_SIGHT if self.tag_is_fresh() else HOVER)
                else:
                    # Never climb blindly forever: without a tag, hold instead.
                    self.transition(ALIGN if self.tag_is_fresh() else TAG_LOST)
        elif not self.servo_active:
            # not servoing: hold position; TAG_IN_SIGHT tells the operator
            # the detector sees the tag, i.e. ibvs/start will work
            if self.state not in (HOVER, TAG_IN_SIGHT):
                self.transition(HOVER)           # e.g. ibvs/stop while servoing
            elif self.state == HOVER and self.tag_is_fresh():
                self.transition(TAG_IN_SIGHT)
            elif self.state == TAG_IN_SIGHT and not self.tag_is_fresh():
                self.transition(HOVER)
        elif self.state in (HOVER, TAG_IN_SIGHT):
            self.transition(ALIGN if self.tag_is_fresh() else TAG_LOST)
        elif self.state in (ALIGN, ALIGNED) and not self.tag_is_fresh():
            self.transition(TAG_LOST)
        elif self.state == TAG_LOST:
            if self.tag_is_fresh():
                self.transition(ALIGN)

        return ready_to_fly

    def compute_thrust(self):
        """Climb-rate command via the thrust field (0.5 = zero climb rate).

        No depth means no standoff to regulate toward, and no depth-free
        descent strategy is enabled yet either (see the README
        limitations) -- centering the target laterally does not by itself
        bring the vehicle any closer. ALIGN/ALIGNED instead hold the
        ODOMETRY height the vehicle was at when servoing started
        (hold_height, latched in transition()) -- NOT takeoff_height,
        which CLIMB may over/undershoot, and not just an open-loop
        hover_thrust, which can drift over an arbitrarily long wait in
        HOVER before ibvs/start is called. Every other non-CLIMB state
        still just sends hover_thrust.
        """
        if self.state == CLIMB:
            return self.climb_thrust

        if (self.state in (ALIGN, ALIGNED) and
                self.hold_height is not None and self.last_odom is not None):
            z_err = self.hold_height - self.last_odom.pose.pose.position.z
            vz = self.last_odom.twist.twist.linear.z
            delta = clamp(self.kp_alt_hold * z_err - self.kv_alt_hold * vz,
                          self.thrust_min - self.hover_thrust,
                          self.thrust_max - self.hover_thrust)
            return self.hover_thrust + delta

        return self.hover_thrust

    def compute_body_rates(self):
        """Cascade: tag position error -> desired tilt -> body rate.

        The attitude loop must stay active in EVERY flying state (including
        ALIGNED and TAG_LOST) -- with IGNORE_ATTITUDE set, a zero body rate
        means "keep the current tilt", which would fly away. Level flight is
        desired_tilt = 0, not rate = 0.
        """
        if self.last_odom is None:
            return 0.0, 0.0

        vel = self.last_odom.twist.twist.linear
        q = self.last_odom.pose.pose.orientation
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Default (CLIMB/HOVER/TAG_LOST): hold the latched position. Level
        # attitude alone is NOT a hover -- attitude trim bias pushes the
        # vehicle sideways and velocity braking only caps the drift, so a
        # position P-term is needed to actually stand still. Same cascade
        # signs as the tag law, with the latched point playing the tag.
        desired_pitch = 0.0
        desired_roll = 0.0
        if self.hold_position is not None and self.state in HOLD_STATES:
            pos = self.last_odom.pose.pose.position
            ex = self.hold_position[0] - pos.x
            ey = self.hold_position[1] - pos.y
            # local ENU error -> body FLU (yaw only)
            cy = math.cos(yaw)
            sy = math.sin(yaw)
            err_bx = cy * ex + sy * ey
            err_by = -sy * ex + cy * ey
            desired_pitch = clamp(self.kp_hover * err_bx - self.kv_hover * vel.x,
                                  -self.max_tilt, self.max_tilt)
            desired_roll = clamp(-self.kp_hover * err_by + self.kv_hover * vel.y,
                                 -self.max_tilt, self.max_tilt)

        if self.state in (ALIGN, ALIGNED) and self.t_x is not None:
            # target error as a frame-fraction (from the pixel point) --
            # this is what drives the PID cascade below, kept
            # resolution-independent on purpose (see module docstring).
            err_x = self.target_x - self.t_x
            err_y = self.target_y - self.t_y

            # The ALIGN/ALIGNED check, however, is done in raw PIXELS: scale
            # each frame-fraction error back out by its own half-dimension
            # to recover the true (possibly non-square) pixel-space
            # distance, e.g. "must be within 20 px of center" regardless of
            # image_width/image_height.
            error_px_x = err_x * (self.image_width / 2.0)
            error_px_y = err_y * (self.image_height / 2.0)
            error_norm_px = (error_px_x ** 2 + error_px_y ** 2) ** 0.5

            if self.state == ALIGN:
                if error_norm_px < self.align_tolerance_px:
                    if self.aligned_since is None:
                        self.aligned_since = rospy.Time.now()
                    elif (rospy.Time.now() - self.aligned_since).to_sec() >= self.align_dwell_time:
                        self.transition(ALIGNED)
                else:
                    self.aligned_since = None
            elif error_norm_px > self.align_tolerance_px * self.align_hysteresis:
                self.transition(ALIGN)

            # We must fly TOWARD the tag. FLU sign conventions:
            #   +pitch = nose down = +x accel  ->  pitch error = tag_x - target_x
            #   +roll  = right down = -y accel ->  roll error  = target_y - tag_y
            desired_pitch = self.pid_x.update(-err_x, -vel.x, self.dt)
            desired_roll = self.pid_y.update(err_y, vel.y, self.dt)

        roll_rate = clamp(self.kp_att * (desired_roll - roll),
                          -self.max_body_rate, self.max_body_rate)
        pitch_rate = clamp(self.kp_att * (desired_pitch - pitch),
                           -self.max_body_rate, self.max_body_rate)
        return roll_rate, pitch_rate

    def control_loop(self, _event):
        self.update_state_machine()
        thrust = self.compute_thrust()
        roll_rate, pitch_rate = self.compute_body_rates()
        self.publish_setpoint(roll_rate, pitch_rate, thrust)

    def publish_setpoint(self, roll_rate, pitch_rate, thrust):
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        msg.body_rate.x = roll_rate
        msg.body_rate.y = pitch_rate
        msg.body_rate.z = 0.0
        msg.thrust = thrust
        self.setpoint_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('ibvs_controller')
    try:
        IbvsController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
