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
  - X-Y: closed-loop IBVS. The setpoint comes ONLY from the detection's
         offset from the image centre (normalized per axis to a
         frame-fraction, +-1.0 at that axis' frame edge, NO camera
         calibration anywhere), and that offset commands a TILT:

             desired_tilt = PID_xy(image error)      (clamped to max_tilt)

         kp is therefore the TILT commanded at the frame edge (rad), not a
         rate. Normalizing each axis by its own half-dimension keeps a 16:9
         frame symmetric: with a single focal length the wide axis reached
         640 px against the short axis' 360 px, so only the wide one could
         saturate (bench tested: pitch pegged while roll topped out at 0.29).

         HOW THE TILT IS SENT is ~command_mode:
           attitude (default) -- publish the target ATTITUDE quaternion and
             let ArduPilot close the attitude loop: 400 Hz, tuned ATC_*
             gains, feedforward. Nothing about the tilt loop runs in Python.
           rate -- publish a body rate from our own inner loop,
             body_rate = kp_att * (desired_tilt - current_tilt), the previous
             behaviour, kept for bench comparison.

         ATTITUDE FEEDBACK comes from mavros/imu/data -- the FCU's own AHRS
         estimate, which needs NO position solution, so it is available with
         no GPS and no OptiTrack. In attitude mode it is needed only for the
         YAW to put in the quaternion (see publish_setpoint); in rate mode it
         closes the tilt loop. (mavros/local_position/odom is not usable for
         either: it requires an EKF POSITION fix and never publishes in this
         setup. It is still read, but only for ALTITUDE -- takeoff_height and
         the landing disarm.)

         WHY COMMANDING A TILT AT ALL -- this is the overshoot fix. A bare
         "rate = kp * error" law leaves tilt as the free INTEGRAL of the
         error: nothing ever commands tilt back toward level, so the vehicle
         reaches the target still tilted and flies straight past it. Only a
         hand-tuned kd could cancel that, and it made the loop third order
             x''' + c*x'' + g*kd*x' + g*kp*x = 0
         whose only x'' damping was aerodynamic drag c, needing c*kd > kp to
         be stable at all -- a knife edge that was flight tested BOTH ways
         (no D term flipped the vehicle; with D it still would not settle).
         Commanding a TILT removes that failure mode structurally:
         desired_tilt is BOUNDED by max_tilt and decays to 0 as the error
         shrinks. kd is no longer a CRASH risk at 0, but it still damps.

         WHY ki IS NON-ZERO -- flight evidence, 2026-08-17-18-30-59.bag. A
         pure-P outer loop cannot hold station against a STEADY disturbance
         (wind, AHRS trim): it needs a standing error big enough to generate
         the balancing tilt, err_ss = (a_dist / g) / kp. At the kp of 0.06
         that bag flew, a modest 0.3 m/s^2 of drift needs HALF THE FRAME of
         standing error -- and that is exactly what the bag shows, the error
         growing to 0.42/-0.59 of the frame over each engagement and staying
         there. It was not diverging; it was sitting at the offset pure P
         implies. ki trims that out (i_max bounds it to ~0.5 m/s^2 worth).

         GAINS ARE ALSO LIMITED BY DETECTION RATE. The D term is computed
         from consecutive detections, so a slow detector both delays the P
         term and blunts the D term. MEASURED across the 2026-08-17 bags:
         3.8-5.9 Hz, median inter-arrival gap 0.157-0.164 s, ~1-2% of gaps
         are multi-second stalls. Raising the detection rate is still the
         single best available improvement.

         MIND ~pid_xy/d_max_dt: it must stay comfortably ABOVE the actual
         detection gap. It shipped at 0.15 s against a measured MEDIAN gap of
         0.157 s -- in 2026-08-17-18-30-59.bag, 92% of gaps exceeded it, so
         the D term was discarded on 92% of detections and kd was very nearly
         inert. At 0.40 s only 1% of that bag's gaps exceed it.

         All in the body FLU convention (mavros converts FLU->FRD for
         MAVLink). Sign conventions (FLU, ROS euler): +pitch = nose down =
         +x accel, +roll = right side down = -y accel.

         AXIS PAIRING: the horizontal image error drives ROLL and the
         vertical error drives PITCH (image right = body RIGHT), which is the
         opposite of the original down-camera assumption (image right = body
         FORWARD). This camera is mounted rotated 90 deg relative to that.
         The per-axis polarity is set by image_x_sign / image_y_sign.

         The outer D term differentiates the DETECTION (computed in
         target_callback from consecutive messages, EMA-filtered by
         ~pid_xy/d_filter), since there is no body-velocity estimate.

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

HOLD STATES AND ATTITUDE: outside ALIGN/ALIGNED there is no detection to
servo on, so desired_tilt is 0 -- and because the attitude loop is closed
on the IMU, that means LEVEL FLIGHT IS ACTIVELY COMMANDED. This is NOT the
same as commanding a zero body rate: with IGNORE_ATTITUDE set a zero rate
means "keep whatever tilt you have", which is what used to leave the
vehicle stuck at its last tilt after a TAG_LOST and let it fly away. The
vehicle now levels itself in WAIT_ARM/CLIMB/HOVER/TAG_IN_SIGHT/TAG_LOST.

Levelling is NOT a position hold: level attitude still drifts on attitude
trim bias, and holding a POINT needs an odometry position estimate this
setup does not have. TAG_LOST is still a cue for the safety pilot to take
over -- it is just a stable, level starting point for that now, instead of
a locked-in tilt.

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

import math

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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

# How the desired tilt reaches the FCU (~command_mode).
#   CMD_ATTITUDE  send the target ATTITUDE (quaternion) and let ArduPilot's
#                 own attitude controller close the loop -- 400 Hz, tuned
#                 ATC_* gains. The default, and the point of this version.
#   CMD_RATE      send a body RATE from our own kp_att inner loop, the
#                 previous behaviour. Kept for bench A/B comparison, and used
#                 automatically until the first IMU message arrives.
CMD_ATTITUDE = 'attitude'
CMD_RATE = 'rate'


def clamp(value, low, high):
    return max(low, min(high, value))


class Pid:
    """Standard PID with output clamp and integral anti-windup.

    The derivative term takes error_dot as a caller-supplied argument (here:
    the differentiated detection, see target_callback) rather than
    differentiating the error signal internally. Whatever is passed as
    `error` and `error_dot` must be a matched pair -- error_dot has to be
    d(error)/dt for the same signal, sign included.
    """

    def __init__(self, kp, ki, kd, out_min, out_max, i_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_max = i_max      # clamp on the INTEGRAL CONTRIBUTION (ki * integral)
        self.integral = 0.0
        # Last individual contributions, kept for the ibvs/pid_* debug topics.
        # These are the terms BEFORE the output clamp, so comparing their sum
        # against max_tilt shows when the clamp is actually biting.
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def reset(self):
        self.integral = 0.0
        self.zero_terms()

    def zero_terms(self):
        """Clear the debug terms -- used when this PID is not being run, so
        the debug topic shows 0 rather than the last value from minutes ago."""
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def update(self, error, error_dot, dt):
        i_term = 0.0
        if self.ki > 0.0:
            self.integral += error * dt
            # anti-windup: keep the integral contribution bounded
            self.integral = clamp(self.integral,
                                  -self.i_max / self.ki, self.i_max / self.ki)
            i_term = self.ki * self.integral

        self.p_term = self.kp * error
        self.i_term = i_term
        self.d_term = self.kd * error_dot

        out = self.p_term + self.i_term + self.d_term
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

        # X-Y axis. The outer (image -> desired tilt) loop is always ours;
        # ~command_mode decides who closes the ATTITUDE loop. See CMD_* above.
        self.command_mode = rospy.get_param('~command_mode', CMD_ATTITUDE)
        if self.command_mode not in (CMD_ATTITUDE, CMD_RATE):
            rospy.logwarn("ibvs_controller: unknown command_mode '%s', "
                          "falling back to '%s'", self.command_mode, CMD_ATTITUDE)
            self.command_mode = CMD_ATTITUDE
        # max_tilt bounds the OUTER loop's output, and is the single most
        # important safety limit here: the vehicle can never be commanded to
        # a steeper attitude than this, however large the pixel error.
        self.max_tilt = rospy.get_param('~max_tilt', 0.15)
        # RATE mode only -- inner loop gain and its rate clamp. Unused in
        # attitude mode, where ArduPilot's ATC_* gains do this job. Keep
        # max_body_rate >= kp_att * max_tilt or that inner loop saturates
        # before it can track a full-scale tilt command.
        self.kp_att = rospy.get_param('~kp_att', 1.5)
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

        # PID gains for the OUTER loop. NOTE the units: kp is rad of TILT per
        # unit of frame-fraction error, so kp IS the tilt commanded at the
        # frame edge (set kp == max_tilt and the frame edge maps to the tilt
        # limit exactly). This is a tilt, NOT a body rate -- the inner
        # attitude loop turns it into one.
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

        # Clamped to max_TILT: these PIDs output a desired attitude now, not
        # a rate. pid_x drives ROLL and pid_y drives PITCH (see the axis
        # pairing note in the module docstring).
        self.pid_x = Pid(kp_xy, ki_xy, kd_xy,
                         -self.max_tilt, self.max_tilt, i_max_xy)
        self.pid_y = Pid(kp_xy, ki_xy, kd_xy,
                         -self.max_tilt, self.max_tilt, i_max_xy)

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
        # Outer-loop damping only -- there is no body-velocity estimate in
        # this setup (no GPS / OptiTrack), and unlike the bare rate law this
        # replaces, the loop no longer DEPENDS on it to be stable.
        self.t_x_dot = 0.0
        self.t_y_dot = 0.0
        self.last_tag_time = None
        # Attitude source for the inner loop. mavros/imu/data is the FCU's
        # AHRS estimate and needs no position solution, so it is available
        # with no GPS and no OptiTrack -- which is exactly why odom cannot be
        # used for this (it requires an EKF POSITION fix and never publishes
        # here). last_odom is still kept, but ONLY for altitude.
        self.last_imu = None
        self.last_odom = None
        # Yaw commanded in attitude mode: latched from the IMU on entering
        # ALIGN so the vehicle holds the heading it engaged at. None means
        # "track the current yaw", i.e. never ask for a yaw change.
        self.yaw_setpoint = None

        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.state_pub = rospy.Publisher('ibvs/state', String, queue_size=1, latch=True)
        # Pixel error the loop is actually working on: detection minus the aim
        # point, in raw pixels, published on every detection so it can be
        # plotted straight against ibvs/target_point and the commanded rates.
        self.error_pub = rospy.Publisher('ibvs/error', PointStamped, queue_size=1)

        # --- debug topics, published every control tick (see publish_debug) ---
        # Individual PID contributions, in RADIANS of desired tilt, one topic
        # per axis: PointStamped x = P term, y = I term, z = D term. There are
        # two independent PIDs and only three slots in a PointStamped, hence
        # two topics rather than one. Their sum is the desired tilt BEFORE the
        # max_tilt clamp, so sum vs max_tilt shows when the clamp is biting.
        self.pid_roll_pub = rospy.Publisher(
            'ibvs/pid_roll', PointStamped, queue_size=1)
        self.pid_pitch_pub = rospy.Publisher(
            'ibvs/pid_pitch', PointStamped, queue_size=1)
        # Attitude actually being commanded, in RADIANS:
        #   x = alpha = roll, y = beta = pitch, z = gamma = yaw
        # This is exactly what goes into the quaternion in attitude mode
        # (after the max_tilt clamp), so it can be plotted straight against
        # mavros/imu/data to see the tracking error.
        self.angles_pub = rospy.Publisher(
            'ibvs/control_angles', PointStamped, queue_size=1)

        # latch the initial state too -- transitions alone would leave the
        # topic silent until the first state change
        self.state_pub.publish(String(data=self.state))

        rospy.Subscriber('mavros/state', State, self.mavros_state_callback, queue_size=1)
        rospy.Subscriber('ibvs/target_point', PointStamped, self.target_callback, queue_size=1)
        rospy.Subscriber('mavros/local_position/odom', Odometry,
                         self.odom_callback, queue_size=1)
        rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback, queue_size=1)

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
        no range sensor). The image_*_sign knobs map the image axes to the
        body frame for the down (land) vs up (perch) camera.

        BOTH AXES ARE NORMALIZED BY THE SAME half-dimension (half_w), NOT each
        by its own. With fx ~ fy, equal ANGULAR offset produces equal PIXEL
        offset on both axes, so dividing y by the shorter half_h would make
        the pitch loop hotter than roll by exactly the aspect ratio
        (640/360 = 1.78x at 1280x720) even though kp is shared between them.
        That asymmetry was measured in 2026-08-18-12-29-58 and -12-32-00: the
        pitch command sat SATURATED at max_tilt in 70-73% of ALIGN samples
        (raw demand averaging 0.077 rad against a 0.035 rad clamp) while roll
        saturated 0-19%, which turned pitch into a relay and produced a
        sustained 2.7-3.5 s limit cycle. Normalizing both by half_w gives the
        two axes the same rad-per-radian gain.

        Consequence: the error is +-1.0 at the LEFT/RIGHT frame edge, and only
        +-image_height/image_width (0.5625 at 16:9) at the TOP/BOTTOM edge.
        lateral_error_px scales BOTH back out by half_w to recover pixels.

        The derivative of the error is taken here, from consecutive
        detections and their real time delta, rather than in the control
        loop -- the detector is slower than control_rate, so differentiating
        per control tick would read zero between detections.
        """
        half_w = self.image_width / 2.0
        half_h = self.image_height / 2.0
        norm_x = (msg.point.x - half_w) / half_w
        norm_y = (msg.point.y - half_h) / half_w

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
                # with kd it dominates the command. Zero it and let P work.
                #
                # d_max_dt must be sized against the REAL detection gap, not
                # guessed: at 0.15 s it sat just below the measured median gap
                # of 0.157 s, so this branch fired on ~80% of detections and
                # silently reduced the loop to P-only. See ~pid_xy/d_max_dt in
                # the config files.
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

    def imu_callback(self, msg):
        self.last_imu = msg

    def current_attitude(self):
        """(roll, pitch, yaw) in body FLU from the FCU's AHRS, or None.

        mavros/imu/data carries the attitude quaternion in the same ENU/FLU
        convention the rest of this node uses, and needs no position
        estimate -- it is available as soon as mavros connects, with no GPS
        and no OptiTrack. Returning None means no IMU message has arrived
        yet; the caller must then command zero rates rather than guess.
        """
        if self.last_imu is None:
            return None
        q = self.last_imu.orientation
        return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

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
                # Latch the heading to hold for the whole approach (attitude
                # mode commands a quaternion, which must carry SOME yaw).
                att = self.current_attitude()
                self.yaw_setpoint = att[2] if att is not None else None
                if self.yaw_setpoint is not None:
                    rospy.loginfo("ibvs_controller: holding yaw %.1f deg for "
                                  "this approach", math.degrees(self.yaw_setpoint))
            elif new_state not in (ALIGN, ALIGNED):
                # not servoing: follow the current heading, never command a change
                self.yaw_setpoint = None
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

        target_callback normalizes BOTH axes by half_w (see the note there),
        so both are scaled back out by half_w to recover pixels. Using
        half_h for y here would under-report the vertical error by the aspect
        ratio and silently loosen align_tolerance_px / descend_xy_gate_px on
        that axis.
        """
        if self.t_x is None:
            return None
        half_w = self.image_width / 2.0
        err_x = (self.target_x - self.t_x) * half_w
        err_y = (self.target_y - self.t_y) * half_w
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

        Height comes from ODOMETRY ALTITUDE (there is no range sensor), and
        that is a hard dependency: mavros/local_position/odom needs an EKF
        POSITION solution, so with no GPS and no OptiTrack this terminal
        CANNOT fire at all -- the descent then runs until the safety pilot
        takes over. The warning below exists so that is never silent.

        The condition must hold for land_disarm_dwell seconds so a single
        spurious low reading cannot disarm mid-air. The disarm is one-shot
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
            # Descending (we are centred in ALIGN/ALIGNED) with no altitude
            # source, so the automatic touchdown disarm can never trigger.
            rospy.logwarn_throttle(
                2.0, "ibvs_controller: DESCENDING but no odometry altitude -- "
                     "automatic land disarm CANNOT fire; the safety pilot must "
                     "take over to stop the descent")
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

    def compute_desired_tilt(self):
        """Outer IBVS loop: image error -> desired (roll, pitch) in rad.

            desired_tilt = PID_xy(image error)       (clamped to max_tilt)

        Pure IBVS: the setpoint comes only from where the detection sits in
        the image -- no calibration, no depth, no attitude feedback in THIS
        stage. How the tilt is then delivered to the FCU is publish_setpoint's
        job and depends on ~command_mode (attitude target, or a body rate via
        the kp_att inner loop).

        Outside ALIGN/ALIGNED there is no detection to servo on, so the
        desired tilt is 0 -- LEVEL FLIGHT, actively commanded. Note this is
        NOT the same as commanding a zero body RATE, which under
        IGNORE_ATTITUDE means "hold the current tilt" and is what used to let
        the vehicle keep flying away after a TAG_LOST.
        """
        # Runs (and drives the ALIGN/ALIGNED transitions) only while servoing
        # on a live detection; every other state wants level.
        desired_roll = 0.0
        desired_pitch = 0.0
        if self.state in (ALIGN, ALIGNED) and self.t_x is not None:
            # Lateral error as a frame-fraction -- this drives the outer law.
            err_x = self.target_x - self.t_x
            err_y = self.target_y - self.t_y

            # The ALIGN/ALIGNED test, however, is done in real PIXELS:
            # scaling each frame-fraction back out by its own half-dimension
            # recovers the true (possibly non-square) pixel distance, so the
            # threshold means "within N px of the aim point" whatever the
            # resolution.
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

            # Axis pairing (this camera is mounted rotated 90 deg from the
            # original down-camera assumption): HORIZONTAL image error drives
            # ROLL, VERTICAL drives PITCH. Polarity per axis is absorbed by
            # image_x_sign / image_y_sign in target_callback. Each PID gets a
            # matched (error, d(error)/dt) pair -- err_x = -t_x, so
            # d(-err_x)/dt = +t_x_dot; err_y = -t_y, so d(err_y)/dt = -t_y_dot.
            desired_roll = self.pid_x.update(-err_x, self.t_x_dot, self.dt)
            desired_pitch = self.pid_y.update(-err_y, self.t_y_dot, self.dt)
        else:
            # PIDs not running: clear their debug terms so ibvs/pid_* reads 0
            # instead of holding whatever it last computed while servoing.
            self.pid_x.zero_terms()
            self.pid_y.zero_terms()

        return desired_roll, desired_pitch

    def tilt_to_body_rates(self, desired_roll, desired_pitch, att):
        """RATE mode inner loop: kp_att * (desired_tilt - measured_tilt).

        Only used when ~command_mode is 'rate'. In 'attitude' mode ArduPilot
        runs this loop itself, at 400 Hz with its own tuned gains, so this
        hand-rolled 30 Hz version is not in the path at all.
        """
        if att is None:
            return 0.0, 0.0
        roll, pitch, _yaw = att
        roll_rate = clamp(self.kp_att * (desired_roll - roll),
                          -self.max_body_rate, self.max_body_rate)
        pitch_rate = clamp(self.kp_att * (desired_pitch - pitch),
                           -self.max_body_rate, self.max_body_rate)
        return roll_rate, pitch_rate

    def control_loop(self, _event):
        self.update_state_machine()
        thrust = self.compute_thrust()
        desired_roll, desired_pitch = self.compute_desired_tilt()
        self.publish_setpoint(desired_roll, desired_pitch, thrust)
        self.publish_debug(desired_roll, desired_pitch)
        if self.mission_mode == MODE_LAND:
            self.maybe_land_disarm()

    def publish_setpoint(self, desired_roll, desired_pitch, thrust):
        """Send the desired tilt to the FCU, as an ANGLE or as a body RATE.

        ATTITUDE mode (~command_mode: attitude, the default) hands the target
        attitude straight to ArduPilot and lets its own attitude controller
        close the loop -- 400 Hz, tuned ATC_* gains, proper feedforward --
        instead of our 30 Hz kp_att P-loop fed by a ~50 Hz IMU topic.
        type_mask ignores the three RATE fields so the quaternion is what
        gets used.

        The quaternion must be NON-ZERO: Copter-Larics-4.3.3's guided mode
        treats an all-zero attitude quaternion as "use body rates" (it routes
        to input_rate_bf_roll_pitch_yaw, see README section 2), so a
        zero-filled orientation would silently select rate control.

        YAW: a quaternion always carries a yaw, and we do not servo yaw. The
        commanded yaw is therefore yaw_setpoint -- latched from the IMU when
        ALIGN is entered, so the vehicle HOLDS the heading it engaged at
        rather than swinging to north (which yaw=0 would command). Outside
        ALIGN/ALIGNED it tracks the current yaw, i.e. never asks for a change.

        RATE mode (~command_mode: rate) is the previous behaviour, kept so the
        two can be compared on the bench without a rebuild. It is also the
        automatic fallback before the first IMU message, since without an
        attitude estimate there is no safe yaw to put in the quaternion.
        """
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        att = self.current_attitude()

        if self.command_mode == CMD_ATTITUDE and att is not None:
            yaw = self.commanded_yaw(att)
            q = tft.quaternion_from_euler(desired_roll, desired_pitch, yaw)
            msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE |
                             AttitudeTarget.IGNORE_PITCH_RATE |
                             AttitudeTarget.IGNORE_YAW_RATE)
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
        else:
            if self.command_mode == CMD_ATTITUDE:
                rospy.logwarn_throttle(
                    5.0, "ibvs_controller: no mavros/imu/data yet -- cannot "
                         "build an attitude target (yaw unknown), falling back "
                         "to zero body rates")
            roll_rate, pitch_rate = self.tilt_to_body_rates(
                desired_roll, desired_pitch, att)
            msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
            msg.body_rate.x = roll_rate
            msg.body_rate.y = pitch_rate
            msg.body_rate.z = 0.0

        msg.thrust = thrust
        self.setpoint_pub.publish(msg)

    def commanded_yaw(self, att):
        """Yaw that goes into the attitude target [rad].

        yaw_setpoint while servoing (latched on ALIGN entry, so the approach
        holds the heading it engaged at); otherwise the current yaw, which
        asks for no change. None if there is no attitude source yet.
        """
        if self.yaw_setpoint is not None:
            return self.yaw_setpoint
        return att[2] if att is not None else None

    def publish_debug(self, desired_roll, desired_pitch):
        """Debug topics, published every control tick.

        ibvs/pid_roll, ibvs/pid_pitch  (PointStamped, RADIANS)
            x = P term, y = I term, z = D term -- the individual contributions
            BEFORE the max_tilt clamp, so their sum against max_tilt shows
            when the clamp is active. One topic per axis: there are two
            independent PIDs and a PointStamped only has three slots. Zero
            outside ALIGN/ALIGNED, where the PIDs do not run.

        ibvs/control_angles  (PointStamped, RADIANS)
            x = alpha = roll, y = beta = pitch, z = gamma = yaw -- the
            attitude actually being commanded, AFTER the clamp. In attitude
            mode this is exactly what the published quaternion encodes, so it
            can be plotted straight against mavros/imu/data to read off the
            tracking error. Yaw is the held heading, not a servoed axis.
        """
        now = rospy.Time.now()

        for pub, pid in ((self.pid_roll_pub, self.pid_x),
                         (self.pid_pitch_pub, self.pid_y)):
            m = PointStamped()
            m.header.stamp = now
            m.point.x = pid.p_term
            m.point.y = pid.i_term
            m.point.z = pid.d_term
            pub.publish(m)

        ang = PointStamped()
        ang.header.stamp = now
        ang.point.x = desired_roll                        # alpha
        ang.point.y = desired_pitch                       # beta
        yaw = self.commanded_yaw(self.current_attitude())
        ang.point.z = yaw if yaw is not None else 0.0     # gamma
        self.angles_pub.publish(ang)


if __name__ == '__main__':
    rospy.init_node('ibvs_controller')
    try:
        IbvsController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
