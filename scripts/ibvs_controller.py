#!/usr/bin/env python

"""
Image-Based Visual Servoing controller for AR-tag perching.

Publishes DIRECTLY to mavros/setpoint_raw/attitude (body rates + thrust),
bypassing the uav_ros_stack tracker/controller entirely. Arming and mode
switching (GUIDED_NOGPS) are done by the startup script, not this node --
this node only ever streams AttitudeTarget setpoints.

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
  - Z:   climb-rate control through the thrust field. During CLIMB
         (takeoff) a fixed climb_thrust is sent. During ALIGN/ALIGNED a
         PID on the tag's relative height regulates toward target_z:
         thrust = hover + PID_z(tag_z - target_z), clamped to
         [thrust_min, thrust_max].
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

    WAIT_ARM --(armed & GUIDED_NOGPS)--> CLIMB
    CLIMB --(climb_settle_time elapsed, servoing NOT started)--> HOVER
    CLIMB --(climb_settle_time elapsed, started & tag seen)--> ALIGN
    CLIMB --(climb_settle_time elapsed, started, NO tag)--> TAG_LOST
    HOVER --(ibvs/start called & tag seen)--> ALIGN
    ALIGN --(|error| < tol for align_dwell_time)--> ALIGNED
    ALIGNED --(|error| > tol * hysteresis)--> ALIGN
    (any state) --(disarmed / mode changed)--> WAIT_ARM
    (any flying state) --(ibvs/stop called)--> HOVER
    (ALIGN/ALIGNED) --(no tag for tag_timeout)--> TAG_LOST
    TAG_LOST --(tag seen again)--> ALIGN

Servoing toward the tag does NOT begin on arming: after takeoff the vehicle
waits in HOVER until the `ibvs/start` service (std_srvs/Trigger) is called
(set ~auto_start to true to skip the gate). `ibvs/stop` returns to HOVER.

Thrust ("climb rate") per state:
    WAIT_ARM  hover_thrust (neutral; ignored anyway while disarmed)
    CLIMB     climb_thrust (constant climb -> this IS the takeoff)
    HOVER     hover_thrust (hold altitude, wait for ibvs/start)
    ALIGN     PID on tag z error, clamped
    ALIGNED   PID on tag z error, clamped
    TAG_LOST  hover_thrust (hold altitude, wait for re-detection)
"""

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


WAIT_ARM = 'WAIT_ARM'
CLIMB = 'CLIMB'
HOVER = 'HOVER'
ALIGN = 'ALIGN'
ALIGNED = 'ALIGNED'
TAG_LOST = 'TAG_LOST'


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

        # Z axis ("climb rate" through the thrust field)
        self.hover_thrust = rospy.get_param('~hover_thrust', 0.5)
        self.climb_thrust = rospy.get_param('~climb_thrust', 0.6)
        self.thrust_min = rospy.get_param('~thrust_min', 0.35)
        self.thrust_max = rospy.get_param('~thrust_max', 0.7)
        self.target_z = rospy.get_param('~target_z', 1.0)

        # X-Y axis (IBVS cascade: PID on tag error -> desired tilt -> body rate)
        self.target_x = rospy.get_param('~target_x', 0.0)
        self.target_y = rospy.get_param('~target_y', 0.0)
        self.max_tilt = rospy.get_param('~max_tilt', 0.15)
        self.kp_att = rospy.get_param('~kp_att', 1.5)
        self.max_body_rate = rospy.get_param('~max_body_rate', 0.35)

        # PID gains (I and D default to 0 -- pure P until tuned otherwise)
        kp_xy = rospy.get_param('~pid_xy/kp', 0.15)
        ki_xy = rospy.get_param('~pid_xy/ki', 0.0)
        kd_xy = rospy.get_param('~pid_xy/kd', 0.0)
        i_max_xy = rospy.get_param('~pid_xy/i_max', 0.05)
        kp_z = rospy.get_param('~pid_z/kp', 0.2)
        ki_z = rospy.get_param('~pid_z/ki', 0.0)
        kd_z = rospy.get_param('~pid_z/kd', 0.0)
        i_max_z = rospy.get_param('~pid_z/i_max', 0.1)

        self.pid_x = Pid(kp_xy, ki_xy, kd_xy, -self.max_tilt, self.max_tilt, i_max_xy)
        self.pid_y = Pid(kp_xy, ki_xy, kd_xy, -self.max_tilt, self.max_tilt, i_max_xy)
        self.pid_z = Pid(kp_z, ki_z, kd_z,
                         self.thrust_min - self.hover_thrust,
                         self.thrust_max - self.hover_thrust, i_max_z)

        # State machine timing / thresholds
        self.climb_settle_time = rospy.get_param('~climb_settle_time', 3.0)
        self.align_tolerance = rospy.get_param('~align_tolerance', 0.15)
        self.align_dwell_time = rospy.get_param('~align_dwell_time', 2.0)
        self.align_hysteresis = rospy.get_param('~align_hysteresis', 1.5)
        self.tag_timeout = rospy.get_param('~tag_timeout', 1.0)

        # Servoing gate: arming only takes off and hovers; flying to the tag
        # starts when the ibvs/start service is called (or ~auto_start: true).
        self.servo_active = rospy.get_param('~auto_start', False)

        self.state = WAIT_ARM
        self.state_entered_at = rospy.Time.now()
        self.aligned_since = None

        self.armed = False
        self.mode = ''
        self.last_tag = None
        self.last_tag_time = None
        self.last_odom = None

        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.state_pub = rospy.Publisher('ibvs/state', String, queue_size=1, latch=True)

        rospy.Subscriber('mavros/state', State, self.mavros_state_callback, queue_size=1)
        rospy.Subscriber('ibvs/tag_pose', PoseStamped, self.tag_callback, queue_size=1)
        rospy.Subscriber('mavros/local_position/odom', Odometry,
                         self.odom_callback, queue_size=1)

        rospy.Service('ibvs/start', Trigger, self.handle_start)
        rospy.Service('ibvs/stop', Trigger, self.handle_stop)

        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

    def handle_start(self, _req):
        self.servo_active = True
        rospy.loginfo("ibvs_controller: servoing STARTED (ibvs/start)")
        return TriggerResponse(success=True, message="IBVS servoing started")

    def handle_stop(self, _req):
        self.servo_active = False
        rospy.loginfo("ibvs_controller: servoing STOPPED (ibvs/stop)")
        return TriggerResponse(success=True, message="IBVS servoing stopped, holding position")

    def mavros_state_callback(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def tag_callback(self, msg):
        self.last_tag = msg
        self.last_tag_time = rospy.Time.now()

    def odom_callback(self, msg):
        self.last_odom = msg

    def transition(self, new_state):
        if new_state != self.state:
            rospy.loginfo("ibvs_controller: %s -> %s", self.state, new_state)
            # entering closed-loop servoing from a non-servoing state:
            # start the PIDs fresh (drops any stale integral)
            if new_state == ALIGN and self.state not in (ALIGN, ALIGNED):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
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
            self.transition(CLIMB)
        elif self.state == CLIMB:
            if self.time_in_state() >= self.climb_settle_time:
                if not self.servo_active:
                    self.transition(HOVER)
                else:
                    # Never climb blindly forever: without a tag, hold instead.
                    self.transition(ALIGN if self.tag_is_fresh() else TAG_LOST)
        elif not self.servo_active:
            # ibvs/stop (or never started): hold position in HOVER
            self.transition(HOVER)
        elif self.state == HOVER:
            self.transition(ALIGN if self.tag_is_fresh() else TAG_LOST)
        elif self.state in (ALIGN, ALIGNED) and not self.tag_is_fresh():
            self.transition(TAG_LOST)
        elif self.state == TAG_LOST:
            if self.tag_is_fresh():
                self.transition(ALIGN)

        return ready_to_fly

    def compute_thrust(self):
        """Climb-rate command via the thrust field (0.5 = zero climb rate)."""
        if self.state == CLIMB:
            return self.climb_thrust

        if self.state in (ALIGN, ALIGNED) and self.last_tag is not None:
            # tag z is the tag's height above the vehicle (body FLU);
            # climb while it is still larger than the desired standoff.
            z_err = self.last_tag.pose.position.z - self.target_z
            z_err_dot = 0.0
            if self.last_odom is not None:
                # stationary tag: d(tag_z)/dt = -climb velocity
                z_err_dot = -self.last_odom.twist.twist.linear.z
            delta = self.pid_z.update(z_err, z_err_dot, self.dt)
            return self.hover_thrust + delta

        # WAIT_ARM (ignored while disarmed) and TAG_LOST: hold altitude.
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

        desired_pitch = 0.0
        desired_roll = 0.0

        if self.state in (ALIGN, ALIGNED) and self.last_tag is not None:
            # tag position error is already in body FLU
            err_x = self.target_x - self.last_tag.pose.position.x
            err_y = self.target_y - self.last_tag.pose.position.y
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

            # Body velocity gives the D-term (odometry twist is body-frame;
            # stationary tag: d(tag_pos)/dt = -velocity).
            vel = self.last_odom.twist.twist.linear

            # We must fly TOWARD the tag. FLU sign conventions:
            #   +pitch = nose down = +x accel  ->  pitch error = tag_x - target_x
            #   +roll  = right down = -y accel ->  roll error  = target_y - tag_y
            desired_pitch = self.pid_x.update(-err_x, -vel.x, self.dt)
            desired_roll = self.pid_y.update(err_y, vel.y, self.dt)

        q = self.last_odom.pose.pose.orientation
        roll, pitch, _ = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

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
