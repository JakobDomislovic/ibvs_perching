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
        point.x  horizontal PIXEL POSITION of the detection in the image,
                 positive RIGHT, from the image origin (top-left)
        point.y  vertical PIXEL POSITION of the detection in the image,
                 positive DOWN, from the image origin (top-left)
        point.z  IGNORED -- there is no range sensor. The vertical axis is
                 open-rate (descend for landing, climb for perching), not
                 target-relative.
    The vision module publishes the POINT it sees and applies no geometry.
    The controller normalizes it against ~image_width / ~image_height, each
    axis by its OWN half-dimension, so the error is +-1.0 at that axis'
    frame edge. NO camera calibration is involved anywhere in the chain.
    ~target_x / ~target_y are the desired offset as a frame-fraction, and
    0.0 is dead centre.

    Raw pixels on the wire on purpose: the commanded body rate is a direct
    function of that difference, so a detection can be read straight off
    `rostopic echo ibvs/target_point` and checked against the rate that
    comes out.
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
  - X-Y: closed-loop IBVS computed ONLY from the detection's offset from
         the image centre. NO attitude feedback of any kind: no IMU, no
         AHRS, no odometry attitude. The pixel offset is normalized per axis
         to a frame-fraction (+-1.0 at the frame edge) and becomes a body
         rate directly:

             body_rate = kp * error + kd * d(error)/dt    (clamped)

         kp is therefore the rate commanded AT the frame edge. Normalizing
         each axis by its own half-dimension is what keeps a 16:9 frame
         symmetric: with a single focal length the wide axis reached 640 px
         against the short axis' 360 px, so only the wide one hit the rate
         clamp (bench tested: pitch pegged at 0.35 while roll topped out at
         0.29 and could never saturate).

         All in the body FLU convention (mavros converts FLU->FRD for
         MAVLink). Sign conventions (FLU, ROS euler): +pitch = nose down =
         +x accel, +roll = right side down = -y accel.

         STABILITY -- read before touching the gains. Commanding a RATE
         proportional to a POSITION error makes tilt the integral of that
         error, i.e. a third-order loop:
             x''' + c*x'' + g*kd*x' + g*kp*x = 0
         The x'' coefficient c is aerodynamic drag ALONE -- the attitude
         term used to supply it. Routh-Hurwitz then requires

             c * kd > kp

         so kd MUST be non-zero and kp must stay well under kd times the
         drag coefficient (order 0.5-1 1/s on a multirotor). An earlier
         version of this law with no D term at all was flight tested and
         flipped the vehicle; that is the failure mode this constraint
         avoids. Start conservative and increase kp slowly.

         The derivative is the differentiated detection (computed in
         target_callback from consecutive messages, EMA-filtered by
         ~pid_xy/d_filter). Odometry body velocity is NOT used: it does not
         exist in the real-world setup (no GPS / OptiTrack).

State machine (this is what makes the controller "modal"):

    WAIT_ARM --(ibvs/takeoff called; armed & GUIDED_NOGPS confirmed)--> CLIMB
    WAIT_ARM --(engage_on_target; pilot selects GUIDED_NOGPS; servoing on)--> ALIGN / TAG_LOST
    CLIMB --(takeoff_height reached, servoing NOT started)--> HOVER / TAG_IN_SIGHT
    CLIMB --(takeoff_height reached, started & tag seen)--> ALIGN
    CLIMB --(takeoff_height reached, started, NO tag)--> TAG_LOST
    HOVER <--(tag detection appears / disappears)--> TAG_IN_SIGHT
    HOVER/TAG_IN_SIGHT --(ibvs/start called & tag seen)--> ALIGN
    ALIGN --(|error| < align_tolerance_px for align_dwell_time)--> ALIGNED
    ALIGNED --(|error| > align_tolerance_px * hysteresis)--> ALIGN
    (any state) --(disarmed / mode changed)--> WAIT_ARM
    (any flying state) --(ibvs/stop called)--> HOVER
    (ALIGN/ALIGNED) --(no tag for tag_timeout)--> TAG_LOST
    TAG_LOST --(tag seen again)--> ALIGN

TAG_IN_SIGHT behaves exactly like HOVER; it is a status distinction for the
operator: the detector currently sees the tag, so `ibvs/start` will engage
immediately. Call ibvs/start when `ibvs/state` shows TAG_IN_SIGHT.

HOLD STATES AND ATTITUDE -- important consequence of having no attitude
source: outside ALIGN/ALIGNED there is no detection to servo on, so the
commanded body rates are zero. With IGNORE_ATTITUDE set, zero rate means
"keep the current tilt", NOT "fly level". The vehicle does not self-level
in WAIT_ARM/CLIMB/HOVER/TAG_IN_SIGHT/TAG_LOST, and there is no longer a
position hold (that needed odometry position + attitude). Losing the tag
mid-approach therefore leaves the vehicle at its last tilt: the safety
pilot's RC mode switch is the recovery path, and TAG_LOST is a cue to take
over, not a stable hover.

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
    The safety pilot flies the vehicle manually (e.g. STABILIZE). Two
    things must both be true before the controller does anything:
        1. servoing enabled  -- `ibvs/start` ('i' on the sim keyboard
           joystick, a joystick button on the real RC), or ~auto_start
        2. the PILOT selects GUIDED_NOGPS on the RC mode switch
    Then the controller goes straight to ALIGN (the vehicle is already
    airborne, CLIMB is skipped). `ibvs/stop` disables servoing again, and
    flipping the RC mode switch away from GUIDED_NOGPS hands control back
    instantly, from any state.

ENGAGEMENT -- THE FCU MODE IS THE PILOT'S:
    This node NEVER switches the flight mode on its own. Only the explicit
    `ibvs/takeoff` service (an operator action) may command GUIDED_NOGPS.
    Seeing a target point does not engage anything, and neither does
    arming.

    It used to work the other way: a fresh target point while armed made
    the controller seize GUIDED_NOGPS itself. That meant arming on the
    bench with the tag in view threw the vehicle into GUIDED_NOGPS
    instantly, which is exactly the surprise this design avoids. The mode
    is now the interlock in both directions -- ArduPilot obeys these
    setpoints only in GUIDED_NOGPS, and only the pilot puts it there.

Thrust ("climb rate") per state:
    WAIT_ARM  hover_thrust (neutral; ignored anyway while disarmed)
    CLIMB     climb_thrust (constant climb -> this IS the takeoff)
    HOVER     hover_thrust (hold altitude, wait for ibvs/start)
    ALIGN     centered: land_descend_thrust (land) / perch_climb_thrust
              (perch); off-center: hover_thrust
    ALIGNED   same as ALIGN (land then disarms once low; perch keeps climbing)
    TAG_LOST  hover_thrust (hold altitude, wait for re-detection)
"""

import rospy
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
        # convention -- set them so a target off to one side
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
        # descend/climb only while laterally centered on the tag, in PIXELS:
        # moving vertically off-center shrinks the camera FOV faster than the
        # X-Y loop converges and the tag falls out of frame (flight-tested).
        # Keep this LARGER than align_tolerance_px, otherwise the vehicle can
        # report ALIGNED and still refuse to descend.
        self.descend_xy_gate_px = rospy.get_param('~descend_xy_gate_px', 60.0)

        # X-Y axis (direct law: target error -> body rate, NO attitude loop)
        self.max_body_rate = rospy.get_param('~max_body_rate', 0.35)

        # Camera RESOLUTION -- the only camera knowledge the controller needs.
        # The vision module publishes the detection's PIXEL POSITION and
        # applies no geometry; the controller normalizes it to a fraction of
        # the half-frame, so the error is +-1.0 at the frame edges on BOTH
        # axes. No focal length, no principal point, NO CALIBRATION: gains
        # and tolerances stay valid if the camera resolution changes.
        #
        # This replaces the earlier fx/fy + depth_guess reconstruction, which
        # made the error a physical distance but needed a calibrated focal
        # length. With fx=fy on a 16:9 frame the x axis could reach 640 px
        # while y stopped at 360 px, so only x hit the rate clamp (bench
        # tested: pitch pegged at 0.35 while roll topped out at 0.29).
        # Normalizing per axis puts both edges at 1.0 and removes that
        # asymmetry.
        self.image_width = rospy.get_param('~image_width', 1280)
        self.image_height = rospy.get_param('~image_height', 720)

        # Desired lateral offset as a frame-fraction; 0.0 = dead centre.
        # The centre is subtracted by the normalization, so 0 IS the centre
        # here (unlike the pixel-aim-point scheme this replaces).
        self.target_x = rospy.get_param('~target_x', 0.0)
        self.target_y = rospy.get_param('~target_y', 0.0)

        # PID gains. NOTE the units: kp is rad/s per unit of frame-fraction
        # error (a BODY RATE), so kp IS the rate commanded at the frame edge.
        kp_xy = rospy.get_param('~pid_xy/kp', 0.15)
        ki_xy = rospy.get_param('~pid_xy/ki', 0.0)
        kd_xy = rospy.get_param('~pid_xy/kd', 0.0)
        i_max_xy = rospy.get_param('~pid_xy/i_max', 0.05)
        # The D term differentiates the detection, so it amplifies pixel
        # jitter. This EMA smooths it (1.0 = no filtering).
        self.d_filter = rospy.get_param('~pid_xy/d_filter', 0.3)
        # Longest detection gap still worth differentiating [s]. Beyond this
        # the derivative is zeroed rather than computed across the gap: a
        # slow or stuttering detector otherwise makes the D term, not the
        # position error, the thing steering the aircraft.
        self.d_max_dt = rospy.get_param('~pid_xy/d_max_dt', 0.15)

        self.pid_x = Pid(kp_xy, ki_xy, kd_xy,
                         -self.max_body_rate, self.max_body_rate, i_max_xy)
        self.pid_y = Pid(kp_xy, ki_xy, kd_xy,
                         -self.max_body_rate, self.max_body_rate, i_max_xy)

        # State machine timing / thresholds
        self.climb_settle_time = rospy.get_param('~climb_settle_time', 3.0)
        # Alignment is judged in real PIXELS, not frame-fractions: each axis'
        # fraction is scaled back out by its own half-dimension so the test is
        # a true (possibly non-square) pixel distance, e.g. "within 40 px of
        # centre" regardless of image_width/image_height.
        self.align_tolerance_px = rospy.get_param('~align_tolerance_px', 40.0)
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
        # STABILIZE) and the SAFETY PILOT decides when the controller takes
        # over, by flipping the RC mode switch to GUIDED_NOGPS. Once armed
        # and in GUIDED_NOGPS with servoing started, the controller goes
        # straight to ALIGN -- no takeoff/climb, it is already airborne.
        #
        # The controller NEVER changes the FCU mode by itself. It used to
        # seize GUIDED_NOGPS as soon as it saw a target point while armed,
        # which meant simply arming on the bench (with the tag in view)
        # threw the vehicle into GUIDED_NOGPS instantly. The mode is the
        # pilot's, and only the explicit ibvs/takeoff service may change it.
        #
        # Seeing a target point does still ENABLE SERVOING (engage_armed),
        # so no button press is needed -- but that only decides what the
        # controller does ONCE THE PILOT has selected GUIDED_NOGPS. Both
        # gates must hold to leave WAIT_ARM, and the mode gate is the
        # pilot's alone. ibvs/stop clears engage_armed so a tag in view
        # cannot re-enable servoing behind the pilot's back; ibvs/start
        # re-arms it.
        self.engage_on_target = rospy.get_param('~engage_on_target', False)
        self.engage_armed = self.engage_on_target


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
        # d(target)/dt, differentiated from consecutive detections (filtered).
        # This is the ONLY damping source in the loop now: there is no
        # attitude feedback, and odometry velocity does not exist in the
        # real-world setup (no GPS / OptiTrack).
        self.t_x_dot = 0.0
        self.t_y_dot = 0.0
        self.last_tag_time = None
        self.last_odom = None
        # (x, y) in the local frame that HOVER/TAG_LOST hold on to
        self.hold_position = None

        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.state_pub = rospy.Publisher('ibvs/state', String, queue_size=1, latch=True)
        # Pixel error the loop is actually working on: detection minus the aim
        # point, in raw pixels, published on every detection so it can be
        # plotted straight against ibvs/target_point and the commanded rates.
        self.error_pub = rospy.Publisher('ibvs/error', PointStamped, queue_size=1)
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
        self.engage_armed = self.engage_on_target
        rospy.loginfo("ibvs_controller: servoing STARTED (ibvs/start) -- "
                      "flip the RC mode switch to GUIDED_NOGPS to engage")
        return TriggerResponse(success=True, message="IBVS servoing started")

    def handle_stop(self, _req):
        self.servo_active = False
        # a target point must not silently re-enable servoing after an
        # explicit stop -- ibvs/start re-arms that
        self.engage_armed = False
        rospy.loginfo("ibvs_controller: servoing STOPPED (ibvs/stop)")
        return TriggerResponse(success=True, message="IBVS servoing stopped, holding position")

    def mavros_state_callback(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def target_callback(self, msg):
        """Vision-module PIXEL POSITION -> lateral error, as a frame-fraction.

        point.x/point.y are where the detection sits in the image, in whole
        pixels (positive right / positive down); point.z is ignored (there is
        no range sensor). Each axis is normalized by its OWN half-dimension,
        so the error is +-1.0 at that axis' frame edge -- independent of the
        resolution, and symmetric between a 16:9 frame's wide and short
        axes. The image_*_sign knobs map the image axes to the body frame for
        the down (land) vs up (perch) camera.

        The derivative of the error is taken here, from consecutive
        detections and their real time delta, rather than in the control
        loop -- the detector is slower than control_rate, so differentiating
        per control tick would read zero between detections.
        """
        half_w = self.image_width / 2.0
        half_h = self.image_height / 2.0
        norm_x = (msg.point.x - half_w) / half_w
        norm_y = (msg.point.y - half_h) / half_h

        t_x = self.image_x_sign * norm_x
        t_y = -self.image_y_sign * norm_y
        now = rospy.Time.now()

        if self.t_x is not None and self.last_tag_time is not None:
            dt = (now - self.last_tag_time).to_sec()
            if dt > self.d_max_dt:
                # Detection gap too large to differentiate: (t_x - prev)/dt
                # across a gap is not a velocity, it is the tag having moved
                # (or the vehicle having flown) while we were blind. Dividing
                # a large jump by a large dt still yields a big number, and
                # with kd it dominates the command -- flight-tested on
                # 2026-08-17-09-37-31.bag, where a 1.8 Hz detector left the
                # D term at ~63% of the commanded rate and pegged it at the
                # clamp for 0.3 s while the error was still 130 px. Decay to
                # zero instead and let P do the work.
                self.t_x_dot = 0.0
                self.t_y_dot = 0.0
            elif dt > 1e-3:
                a = self.d_filter
                self.t_x_dot = (1.0 - a) * self.t_x_dot + a * (t_x - self.t_x) / dt
                self.t_y_dot = (1.0 - a) * self.t_y_dot + a * (t_y - self.t_y) / dt

        self.t_x = t_x
        self.t_y = t_y
        self.last_tag_time = now

        # pixel error: detection minus the aim point, in the IMAGE frame
        # (no signs, no normalization) -- positive x = right of centre,
        # positive y = below centre, exactly as the detector reports them.
        # z is always 0: there is no depth here, only a 2-D image offset.
        err = PointStamped()
        err.header.stamp = now
        err.header.frame_id = msg.header.frame_id
        err.point.x = msg.point.x - half_w
        err.point.y = msg.point.y - half_h
        err.point.z = 0.0
        self.error_pub.publish(err)

        # A target point ENABLES SERVOING, but never touches the FCU mode --
        # that belongs to the safety pilot. The controller still only acts
        # once mavros/state reports armed + GUIDED_NOGPS, which the pilot
        # selects on the RC switch (see update_state_machine).
        if self.engage_armed and self.armed and not self.servo_active:
            self.servo_active = True
            rospy.loginfo("ibvs_controller: target point while armed -> "
                          "servoing ENABLED (waiting for the pilot to select "
                          "GUIDED_NOGPS; mode NOT touched)")

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
                # drop the stale derivative too: the detection gap across a
                # TAG_LOST would otherwise show up as a huge d(error)/dt
                self.t_x_dot = 0.0
                self.t_y_dot = 0.0
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
            # airborne, skip CLIMB and servo right away. The pilot selecting
            # GUIDED_NOGPS is the ONLY trigger -- the controller never takes
            # the mode itself.
            #
            # engage_armed (not servo_active) is the gate, and it is true from
            # startup whenever engage_on_target is set. Requiring servo_active
            # here made engagement ORDER-DEPENDENT: servo_active is set by a
            # target point while armed, so if the pilot selected GUIDED_NOGPS
            # BEFORE the first detection arrived, this branch never fired and
            # the state sat in WAIT_ARM with the rates at exactly zero
            # (2026-08-17-09-58-50.bag: GUIDED_NOGPS at +43.9s, first
            # detection at +56.4s, 5.7s after the pilot had already given up
            # and gone back to STABILIZE). Now the mode switch alone engages,
            # and a missing detection simply means TAG_LOST until one arrives.
            elif self.engage_on_target and self.engage_armed:
                self.servo_active = True
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

    def lateral_error_px(self):
        """Distance of the tracked point from the aim point, in real PIXELS,
        or None without a detection.

        t_x/t_y are frame-fractions, so each is scaled back out by its own
        half-dimension to recover a true pixel distance (the two axes have
        different half-dimensions on a non-square frame).
        """
        if self.t_x is None:
            return None
        err_x = (self.target_x - self.t_x) * (self.image_width / 2.0)
        err_y = (self.target_y - self.t_y) * (self.image_height / 2.0)
        return (err_x ** 2 + err_y ** 2) ** 0.5

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
                lateral_error = self.lateral_error_px()
                if lateral_error is not None and lateral_error <= self.descend_xy_gate_px:
                    return min(self.perch_climb_thrust, self.thrust_max)
            return self.hover_thrust

        # LAND: constant slow descent toward the target below, then disarm.
        # Descend ONLY while laterally centered -- descending off-center drops
        # the target out of the shrinking FOV (flight-tested funnel).
        if self.state in (ALIGN, ALIGNED):
            lateral_error = self.lateral_error_px()
            centered = (lateral_error is not None and
                        lateral_error <= self.descend_xy_gate_px)
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

        lateral_error = self.lateral_error_px()
        if lateral_error is None or lateral_error > self.descend_xy_gate_px:
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
        """Direct law: target error -> body rate. NO attitude feedback.

        The rate is computed purely from where the detection sits relative
        to the image centre -- nothing in this path reads the IMU, the AHRS
        or any attitude estimate. The pixel offset is normalized per axis to
        a frame-fraction in target_callback; here it becomes a body rate
        directly:

            body_rate = kp * error + kd * d(error)/dt      (clamped)

        Outside ALIGN/ALIGNED there is no detection to servo on -- by
        definition, that is what those states mean -- so the rates are zero.
        NOTE: with IGNORE_ATTITUDE set, a zero body rate means "keep the
        current tilt", NOT "fly level". The vehicle therefore does not
        self-level in the hold states; the safety pilot's RC mode switch is
        what recovers it.
        """
        if self.state not in (ALIGN, ALIGNED) or self.t_x is None:
            return 0.0, 0.0

        # Lateral error as a frame-fraction -- this is what drives the law.
        err_x = self.target_x - self.t_x
        err_y = self.target_y - self.t_y

        # The ALIGN/ALIGNED test, however, is done in real PIXELS: scaling
        # each frame-fraction back out by its own half-dimension recovers the
        # true (possibly non-square) pixel distance, so the threshold means
        # "within N px of the aim point" whatever the resolution.
        error_norm_px = self.lateral_error_px()

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
        #   +pitch rate = nose down = +x accel -> pitch error = tag_x - target_x
        #   +roll  rate = right down = -y accel -> roll error = target_y - tag_y
        # The D term opposes the error's rate of change (t_*_dot is the
        # differentiated detection; d(err_x)/dt = +d(t_x)/dt with these signs).
        pitch_rate = self.pid_x.update(-err_x, self.t_x_dot, self.dt)
        roll_rate = self.pid_y.update(err_y, -self.t_y_dot, self.dt)

        return (clamp(roll_rate, -self.max_body_rate, self.max_body_rate),
                clamp(pitch_rate, -self.max_body_rate, self.max_body_rate))

    def control_loop(self, _event):
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
        #msg.body_rate.x = roll_rate
        #msg.body_rate.y = pitch_rate
        msg.body_rate.x = -1 * pitch_rate
        msg.body_rate.y =  roll_rate

        msg.body_rate.z = 0.0
        msg.thrust = 0.5 #thrust #samo za probu
        self.setpoint_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('ibvs_controller')
    try:
        IbvsController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
