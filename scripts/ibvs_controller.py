#!/usr/bin/env python

"""
Image-Based Visual Servoing controller.

Publishes DIRECTLY to mavros/setpoint_raw/attitude (body rates + thrust),
bypassing the uav_ros_stack tracker/controller entirely. Arming and mode
switching (GUIDED_NOGPS) are done by the ibvs/takeoff service -- this node
only ever streams AttitudeTarget setpoints.

VISION MODULE INTERFACE (topic `ibvs/target_point`, geometry_msgs/PointStamped):
    The controller is agnostic to WHAT is being tracked. Any vision module
    (ArUco today, anything else tomorrow) publishes the point it wants
    centered in the camera image:
        point.x  normalized horizontal offset from the image center,
                 (u - cx) / fx, positive RIGHT in the image
        point.y  normalized vertical offset from the image center,
                 (v - cy) / fy, positive DOWN in the image
        point.z  IGNORED -- there is no range sensor. The vertical axis is
                 open-rate (descend for landing, climb for perching), not
                 target-relative.
    Publishing on this topic at all means "target in sight": the state
    machine shows TAG_IN_SIGHT and ibvs/start will engage. The controller
    steers so the point goes to the image center (target_x/target_y offsets
    are available).

    The camera mount is selected by mission_mode (land = down, perch = up);
    the image->body axis mapping is set by image_x_sign / image_y_sign so
    the same interface serves either orientation.

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
  - Z:   OPEN-RATE climb control through the thrust field (no range sensor).
         During CLIMB (takeoff) a fixed climb_thrust is sent. During
         ALIGN/ALIGNED, while laterally centered, a fixed rate is sent:
         land_descend_thrust (< 0.5, descend) in land mode or
         perch_climb_thrust (> 0.5, climb) in perch mode; off-center or in
         any hold state it is hover_thrust. Landing then disarms on odometry
         altitude (land_disarm_height); perching is finished by the pilot.
  - X-Y: closed-loop IBVS as a CASCADE. A raw body-rate command
         proportional to position error is unstable (the rate integrates
         into an unbounded tilt -- flight tested, it flips the vehicle),
         so instead:
             desired_tilt = PID_xy(tag position error)   (clamped small)
             body_rate    = kp_att * (desired_tilt - current_tilt)
         Attitude and body velocity are taken from odometry. All in the
         body FLU convention (mavros converts FLU->FRD for MAVLink).
         Sign conventions (FLU, ROS euler): +pitch = nose down = +x accel,
         +roll = right side down = -y accel.

         The PID derivative term uses body velocity (d(err)/dt = -vel for
         a stationary tag) instead of numerically differentiating the
         detection -- same information, far less noise. With kd > 0 it is
         exactly the velocity damping term of a classic tilt cascade.

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

RC MASTER GATE (~rc_gate_enabled: true, real world):
    A hardware switch on the transmitter is the master enable, read
    straight from mavros/rc/in (channel ~rc_gate_channel, 0-based; ON when
    above ~rc_gate_threshold, optionally ~rc_gate_invert). While the switch
    is OFF the controller publishes NOTHING to the FCU -- it is completely
    inert, so no setpoints reach ArduPilot until the pilot flips the switch.
    Flipping it ON is the engage trigger (equivalent to ibvs/start): it
    primes engagement, then a fresh target point switches the FCU to
    GUIDED_NOGPS and servoing begins. Flipping it OFF mid-flight stops all
    setpoints and disengages -- take manual control with the RC mode switch.
    Fails safe: absent or stale RC (older than ~rc_gate_timeout) reads OFF.

Thrust ("climb rate") per state:
    WAIT_ARM  hover_thrust (neutral; ignored anyway while disarmed)
    CLIMB     climb_thrust (constant climb -> this IS the takeoff)
    HOVER     hover_thrust (hold altitude, wait for ibvs/start)
    ALIGN     centered: land_descend_thrust (land) / perch_climb_thrust
              (perch); off-center: hover_thrust
    ALIGNED   same as ALIGN (land then disarms once low; perch keeps climbing)
    TAG_LOST  hover_thrust (hold altitude, wait for re-detection)
"""

import math

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, RCIn, State
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

# Mission mode -- what the vertical (thrust/climb-rate) axis does once the
# controller is centered on the target. Both modes center the target in the
# image the same way (X-Y IBVS cascade); they differ only in the vertical:
#   MODE_LAND  down-facing camera, target BELOW: descend onto it, then disarm
#              (real touchdown). Uses the range hint (point.z) as height.
#   MODE_PERCH up-facing camera, branch ABOVE: climb toward it while centered.
#              Range is ignored; there is no automatic terminal -- the safety
#              pilot takes over manually once the vehicle is at the branch.
MODE_LAND = 'land'
MODE_PERCH = 'perch'


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

        # Mission mode: 'land' (down cam, descend + disarm) or 'perch'
        # (up cam, climb toward the branch). See MODE_* above.
        self.mission_mode = rospy.get_param('~mission_mode', MODE_LAND)
        if self.mission_mode not in (MODE_LAND, MODE_PERCH):
            rospy.logwarn("ibvs_controller: unknown mission_mode '%s', "
                          "falling back to '%s'", self.mission_mode, MODE_LAND)
            self.mission_mode = MODE_LAND

        # Image->body sign per axis. The vision interface is fixed (point.x
        # positive RIGHT, point.y positive DOWN) but the physical mount is
        # not: a down camera (landing) and an up camera (perching) map the
        # same image error to OPPOSITE body directions. These two knobs
        # absorb both the camera flip and the detector's pixel-sign
        # convention -- set them on the bench so a target off to one side
        # produces a correction TOWARD it (negative feedback). Defaults suit
        # the down-facing landing camera.
        self.image_x_sign = rospy.get_param('~image_x_sign', 1.0)
        self.image_y_sign = rospy.get_param('~image_y_sign', 1.0)

        # PERCH: constant climb-rate command sent while centered on the
        # branch above (> hover_thrust = climb). Held to thrust_max.
        self.perch_climb_thrust = rospy.get_param('~perch_climb_thrust', 0.55)

        # LAND: constant slow descent command sent while centered
        # (< hover_thrust = descend). Held to thrust_min. There is no range
        # sensor -- the descent is open-rate and the terminal disarm uses
        # odometry altitude (see maybe_land_disarm).
        self.land_descend_thrust = rospy.get_param('~land_descend_thrust', 0.45)

        # LAND: cut thrust and disarm once centered and this low, using
        # odometry altitude. land_disarm_dwell debounces spurious readings.
        self.disarm_on_land = rospy.get_param('~disarm_on_land', True)
        self.land_disarm_height = rospy.get_param('~land_disarm_height', 0.15)
        self.land_disarm_dwell = rospy.get_param('~land_disarm_dwell', 0.3)
        self.landed = False
        self._land_ready_since = None

        # Z axis ("climb rate" through the thrust field)
        self.hover_thrust = rospy.get_param('~hover_thrust', 0.5)
        self.climb_thrust = rospy.get_param('~climb_thrust', 0.6)
        self.thrust_min = rospy.get_param('~thrust_min', 0.35)
        self.thrust_max = rospy.get_param('~thrust_max', 0.7)
        self.takeoff_height = rospy.get_param('~takeoff_height', 2.0)
        # fixed scale from the normalized image offset to the pseudo body
        # position error -- there is no depth sensor, so this constant just
        # sets the X-Y loop gain together with pid_xy (never used for Z)
        self.depth_guess = rospy.get_param('~depth_guess', 2.0)
        # descend/climb only while laterally centered on the tag: moving
        # vertically off-center shrinks the camera FOV faster than the X-Y
        # loop converges and the tag falls out of frame (flight-tested)
        self.descend_xy_gate = rospy.get_param('~descend_xy_gate', 0.25)

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

        # PID gains (I and D default to 0 -- pure P until tuned otherwise).
        # X-Y only: the vertical axis is open-rate (no range sensor), so there
        # is no Z PID.
        kp_xy = rospy.get_param('~pid_xy/kp', 0.15)
        ki_xy = rospy.get_param('~pid_xy/ki', 0.0)
        kd_xy = rospy.get_param('~pid_xy/kd', 0.0)
        i_max_xy = rospy.get_param('~pid_xy/i_max', 0.05)

        self.pid_x = Pid(kp_xy, ki_xy, kd_xy, -self.max_tilt, self.max_tilt, i_max_xy)
        self.pid_y = Pid(kp_xy, ki_xy, kd_xy, -self.max_tilt, self.max_tilt, i_max_xy)

        # State machine timing / thresholds
        self.climb_settle_time = rospy.get_param('~climb_settle_time', 3.0)
        self.align_tolerance = rospy.get_param('~align_tolerance', 0.15)
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

        # RC gate: a hardware switch on the transmitter is the master enable.
        # While OFF the controller publishes NOTHING to the FCU (fully inert);
        # flipping it ON is the "start" trigger (primes engagement, then a
        # fresh target point engages). Read straight from mavros/rc/in so it
        # does not depend on rc_to_joy. Fails safe: stale/absent RC = OFF.
        # rc_gate_channel is the 0-based index into RCIn.channels (channel N
        # on the transmitter is index N-1). Disabled by default so the sim /
        # keyboard flow (ibvs/start) is unchanged.
        self.rc_gate_enabled = rospy.get_param('~rc_gate_enabled', False)
        self.rc_gate_channel = int(rospy.get_param('~rc_gate_channel', 6))
        self.rc_gate_threshold = float(rospy.get_param('~rc_gate_threshold', 1500.0))
        self.rc_gate_invert = rospy.get_param('~rc_gate_invert', False)
        self.rc_gate_timeout = float(rospy.get_param('~rc_gate_timeout', 0.5))
        self.last_rc = None
        self.last_rc_time = None
        self._rc_gate_prev = False

        self.state = WAIT_ARM
        self.state_entered_at = rospy.Time.now()
        self.aligned_since = None

        self.armed = False
        self.mode = ''
        # target reconstructed from the vision module's image point, in the
        # body FLU frame: t_x forward, t_y left. There is no depth axis --
        # the vertical (climb/descend) is open-rate, not target-relative.
        self.t_x = None
        self.t_y = None
        self.last_tag_time = None
        self.last_odom = None
        # (x, y) in the local frame that HOVER/TAG_LOST hold on to
        self.hold_position = None

        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.state_pub = rospy.Publisher('ibvs/state', String, queue_size=1, latch=True)
        # latch the initial state too -- transitions alone would leave the
        # topic silent until the first state change
        self.state_pub.publish(String(data=self.state))

        rospy.Subscriber('mavros/state', State, self.mavros_state_callback, queue_size=1)
        rospy.Subscriber('mavros/rc/in', RCIn, self.rc_callback, queue_size=1)
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
        self.landed = False
        self._land_ready_since = None
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
        self.landed = False
        self._land_ready_since = None
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

    def rc_callback(self, msg):
        self.last_rc = msg.channels
        self.last_rc_time = rospy.Time.now()

    def rc_gate_is_on(self):
        """Master enable from the RC switch. True = IBVS may command the FCU.

        Disabled (~rc_gate_enabled false) -> always True (sim/keyboard flow).
        Enabled -> reads mavros/rc/in and fails SAFE: no fresh RC, too few
        channels, or the switch below threshold all read as OFF.
        """
        if not self.rc_gate_enabled:
            return True
        if self.last_rc_time is None or \
                (rospy.Time.now() - self.last_rc_time).to_sec() > self.rc_gate_timeout:
            return False
        if self.last_rc is None or len(self.last_rc) <= self.rc_gate_channel:
            return False
        on = self.last_rc[self.rc_gate_channel] > self.rc_gate_threshold
        return (not on) if self.rc_gate_invert else on

    def target_callback(self, msg):
        """Vision-module point -> pseudo target position in body FLU.

        Only the normalized image offset (point.x/point.y) is used; point.z
        is ignored (there is no range sensor). The offset is scaled by the
        fixed depth_guess into a body X-Y error for the centering loop; the
        image_*_sign knobs map the image axes to the body frame for the
        down (land) vs up (perch) camera.
        """
        depth = self.depth_guess
        self.t_x = depth * self.image_x_sign * msg.point.x
        self.t_y = -depth * self.image_y_sign * msg.point.y
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
            # start the PIDs fresh (drops any stale integral)
            if new_state == ALIGN and self.state not in (ALIGN, ALIGNED):
                self.pid_x.reset()
                self.pid_y.reset()
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

    def lateral_error(self):
        """Distance of the tracked point from the desired image center [m],
        in the body frame, or None without a detection."""
        if self.t_x is None:
            return None
        return ((self.target_x - self.t_x) ** 2 +
                (self.target_y - self.t_y) ** 2) ** 0.5

    def compute_thrust(self):
        """Climb-rate command via the thrust field (0.5 = zero climb rate)."""
        if self.state == CLIMB:
            return self.climb_thrust

        # PERCH: the branch is ABOVE (up camera). Climb toward it, but only
        # while laterally centered -- climbing off-center drifts the branch
        # out of the shrinking FOV, same funnel logic as the landing descent.
        # Range is ignored; the safety pilot commits the final grab manually.
        if self.mission_mode == MODE_PERCH:
            if self.state in (ALIGN, ALIGNED):
                lateral_error = self.lateral_error()
                if lateral_error is not None and lateral_error <= self.descend_xy_gate:
                    return min(self.perch_climb_thrust, self.thrust_max)
            return self.hover_thrust

        # LAND: constant slow descent toward the target below, then disarm.
        # Descend ONLY while laterally centered -- descending off-center drops
        # the target out of the shrinking FOV (flight-tested funnel).
        if self.state in (ALIGN, ALIGNED):
            lateral_error = self.lateral_error()
            centered = (lateral_error is not None and
                        lateral_error <= self.descend_xy_gate)
            if centered:
                return max(self.land_descend_thrust, self.thrust_min)
            return self.hover_thrust

        # WAIT_ARM (ignored while disarmed) and TAG_LOST: hold altitude.
        return self.hover_thrust

    def maybe_land_disarm(self):
        """LAND terminal: once centered and low enough, disarm (touchdown).

        Height comes from odometry altitude (there is no range sensor). The
        condition must hold for land_disarm_dwell seconds so a single spurious
        low reading cannot disarm mid-air. The disarm is one-shot
        (self.landed); a fresh ibvs/takeoff or ibvs/start re-arms it.
        """
        if self.landed or not self.disarm_on_land:
            return
        if self.state not in (ALIGN, ALIGNED):
            self._land_ready_since = None
            return

        lateral_error = self.lateral_error()
        if lateral_error is None or lateral_error > self.descend_xy_gate:
            self._land_ready_since = None
            return

        if self.last_odom is None:
            self._land_ready_since = None
            return
        height = self.last_odom.pose.pose.position.z

        if height > self.land_disarm_height:
            self._land_ready_since = None
            return

        now = rospy.Time.now()
        if self._land_ready_since is None:
            self._land_ready_since = now
            return
        if (now - self._land_ready_since).to_sec() < self.land_disarm_dwell:
            return

        self.landed = True
        self.servo_active = False
        rospy.loginfo("ibvs_controller: LANDED (height %.2f m <= %.2f) -- disarming",
                      height, self.land_disarm_height)
        try:
            self.arming_srv(False)
        except rospy.ServiceException as exc:
            rospy.logerr("ibvs_controller: disarm failed: %s", exc)

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
            # target position error in body FLU (from the image point)
            err_x = self.target_x - self.t_x
            err_y = self.target_y - self.t_y
            error_norm = (err_x ** 2 + err_y ** 2) ** 0.5

            if self.state == ALIGN:
                if error_norm < self.align_tolerance:
                    if self.aligned_since is None:
                        self.aligned_since = rospy.Time.now()
                    elif (rospy.Time.now() - self.aligned_since).to_sec() >= self.align_dwell_time:
                        self.transition(ALIGNED)
                else:
                    self.aligned_since = None
            elif error_norm > self.align_tolerance * self.align_hysteresis:
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
        # RC master gate: while the switch is OFF the controller is inert and
        # sends NOTHING to the FCU. Flipping it ON is the engage trigger.
        gate_on = self.rc_gate_is_on()
        if not gate_on:
            if self._rc_gate_prev:
                rospy.loginfo("ibvs_controller: RC gate OFF -- IBVS disengaged, "
                              "no setpoints sent")
                self.servo_active = False
                self.engage_armed = False
                self._land_ready_since = None
                self.transition(WAIT_ARM)
            self._rc_gate_prev = False
            return
        if not self._rc_gate_prev:
            # rising edge: switch flipped ON -- prime engagement (== ibvs/start)
            rospy.loginfo("ibvs_controller: RC gate ON -- IBVS live, awaiting target")
            self.servo_active = True
            self.engage_armed = self.engage_on_target
            self.landed = False
            self._land_ready_since = None
            self._rc_gate_prev = True

        self.update_state_machine()
        thrust = self.compute_thrust()
        roll_rate, pitch_rate = self.compute_body_rates()
        self.publish_setpoint(roll_rate, pitch_rate, thrust)
        if self.mission_mode == MODE_LAND:
            self.maybe_land_disarm()

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
