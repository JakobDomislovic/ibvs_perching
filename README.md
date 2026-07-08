# ibvs_perching

**Image-Based Visual Servoing (IBVS) for AR-tag perching with direct body-rate
control through MAVROS.**

This package implements a minimal, modal visual-servoing controller for a UAV
approaching an AR tag from below. Unlike the rest of the `perching_uav` /
`uav_ros_stack` pipeline, it does **not** use the position tracker or the
`control_manager` controller (MPC / carrot / cascade PID). It commands
ArduPilot **directly** over `mavros/setpoint_raw/attitude`, sending roll/pitch
**body rates** for lateral (X-Y) alignment and a **climb-rate** command
(through the `thrust` field) for the vertical (Z) approach.

---

## Table of contents

1. [Motivation & design decisions](#1-motivation--design-decisions)
2. [How ArduPilot interprets the setpoint (REAErik vnuc
MartaD THIS)](#2-how-ardupilot-interprets-the-setpoint-read-this)
3. [Architecture](#3-architecture)
4. [The state machine](#4-the-state-machine)
5. [Control laws](#5-control-laws)
6. [Nodes, topics & parameters](#6-nodes-topics--parameters)
7. [Building](#7-building)
8. [Running the simulation demo](#8-running-the-simulation-demo)
9. [Troubleshooting (incl. "it does not take off")](#9-troubleshooting)
10. [Integrating a real AR-tag detector](#10-integrating-a-real-ar-tag-detector)
11. [Known limitations & future work](#11-known-limitations--future-work)

---

## 1. Motivation & design decisions

The perching scenario: the UAV sits below a structure carrying an AR tag. It
must **climb toward the tag** while **aligning itself in X-Y** so that it ends
up directly underneath, at a chosen standoff distance, ready to perch.

Key decisions and their rationale:

| Decision | Rationale |
|---|---|
| Bypass the `uav_ros_stack` tracker/controller | IBVS closes the loop on the *visual feature error* directly, at body-rate level. Feeding Cartesian setpoints through the position tracker would add a slow outer loop that isn't needed and obscures the visual dynamics. |
| Publish to `mavros/setpoint_raw/attitude` | Lowest-level setpoint interface MAVROS offers for guided flight: body rates + thrust/climb-rate, mapped to the MAVLink `SET_ATTITUDE_TARGET` message. |
| Z axis via climb rate, hover at `0.5` | ArduPilot interprets the `thrust` field as a climb-rate command in guided modes (see §2), giving us a well-damped, firmware-stabilized vertical channel for free. |
| Modal (state machine) design | Perching is inherently phased: take off, acquire the tag, align, hold. Explicit states make the behavior predictable, debuggable (`ibvs/state` topic), and safe (dedicated `TAG_LOST` and disarm handling). |
| Separate mock detector node | The controller only sees a `PoseStamped` on `ibvs/tag_pose`. Swapping the mock for a real detector requires zero controller changes. |

## 2. How ArduPilot interprets the setpoint (READ THIS)

`mavros_msgs/AttitudeTarget` is sent as MAVLink `SET_ATTITUDE_TARGET`. We set

```
type_mask = IGNORE_ATTITUDE (128)   # use body_rate + thrust, ignore orientation
```

**The `thrust` field is NOT motor thrust.** In GUIDED / GUIDED_NOGPS,
ArduCopter interprets `thrust` as a **normalized climb-rate command** — unless
`GUID_OPTIONS` bit 3 ("SetAttitudeTarget interprets Thrust as Thrust") is set,
which it is not by default:

| `thrust` value | Commanded vertical motion |
|---|---|
| `0.0` | descend at maximum rate (`PILOT_SPEED_DN`) |
| `0.5` | **zero climb rate — hold altitude ("hover")** |
| `1.0` | climb at maximum rate (`PILOT_SPEED_UP`) |

Two practical consequences:

1. **Takeoff requires `thrust > 0.5`.** Sending a constant `0.5` arms the
   motors at ground idle and the vehicle never leaves the ground (and
   auto-disarms after `DISARM_DELAY`, default ~10 s). This is exactly the
   classic "it does not want to take off" symptom. The controller therefore
   sends `climb_thrust` (default `0.6`) during its `CLIMB` state.
2. **`0.5` is the safe neutral value**, so it is what the controller sends
   when idle (`WAIT_ARM`) and when the tag is lost (`TAG_LOST`).

> **⚠ CRITICAL — `GUID_OPTIONS` on the kopterworx:** the LARICS
> `identity.parm` (loaded by SITL *and* flashed on the real vehicles) sets
> **`GUID_OPTIONS = 8`**, i.e. thrust-as-raw-thrust, because the
> `uav_ros_stack` MPC computes true thrust through its thrust model. With
> that setting this controller's commands (0.35–0.7) are all above the
> vehicle's hover throttle (`MOT_THST_HOVER ≈ 0.29`) and it **flies away at
> a constant climb** (flight-tested: ~5.4 m/s, straight past 300 m). The
> `startup/sim_ibvs` session therefore runs
> `rosrun mavros mavparam set GUID_OPTIONS 0` before launching the
> controller — do the same on any vehicle before flying this package, and
> set it back for the normal MPC stack.

The roll/pitch channels (`body_rate.x/y`) are genuine body-frame angular-rate
commands in rad/s, tracked by ArduPilot's rate controllers. (The official
"Copter Commands in Guided Mode" docs claim body rates are unsupported —
that is outdated for this fork: `Copter-Larics-4.3.3`'s guided mode routes a
zero attitude quaternion to `input_rate_bf_roll_pitch_yaw()`, i.e. true
body-rate control; see `ArduCopter/mode_guided.cpp`.)

## 3. Architecture

```
                                 +--------------------------+
   mavros/local_position/odom -->|  mock_ar_tag_publisher   |
   (real UAV odometry)           |  (fakes a detector from  |
                                 |   a fixed tag world pos) |
                                 +-----------+--------------+
                                             | ibvs/tag_pose (PoseStamped,
                                             |  tag position in body FLU)
                                             v
   mavros/state ------------->  +--------------------------+
   (armed? GUIDED_NOGPS?)       |     ibvs_controller      |--> ibvs/state (String, latched)
                                |  modal state machine +   |
                                |  P-control on tag error  |--> mavros/setpoint_raw/attitude
                                +--------------------------+    (AttitudeTarget @ control_rate)
                                                                     |
                                                                     v
                                                    MAVROS -> ArduPilot (SET_ATTITUDE_TARGET)
```

Everything runs under the UAV namespace (`$UAV_NAMESPACE`, default `red`), so
topic names above are relative (`/red/ibvs/tag_pose`, etc).

The tag pose convention is **body FLU**: `x` forward, `y` left, `z` up. So a
tag at `(-2, 0, 1)` is 2 m *behind* and 1 m *above* the vehicle.

## 4. The state machine

```
             armed & GUIDED_NOGPS            settle time & tag fresh
  WAIT_ARM ------------------------> CLIMB ---------------------------> ALIGN
     ^                                 |                                 |  ^
     |                                 | settle time, NO tag             |  |
     |  disarm / mode change           v                                 |  | error > tol*hyst
     +---------- (from ANY state) TAG_LOST <---- tag stale (timeout) ----+  |
                                       |                                 v  |
                                       +------ tag fresh again ------> ALIGNED
                                                            (|err| < tol for dwell time)
```

| State | Meaning | desired tilt (X-Y) | thrust (climb rate) |
|---|---|---|---|
| `WAIT_ARM` | waiting for `armed && mode == GUIDED_NOGPS` | level (0) | `hover_thrust` (neutral; ignored while disarmed) |
| `CLIMB` | open-loop takeoff / climb phase | level (0) | `climb_thrust` (**> 0.5 → the vehicle lifts off**) |
| `ALIGN` | closed-loop X-Y servoing + Z standoff regulation | cascade on tag error | P-control on tag height |
| `ALIGNED` | error small & settled — hold | cascade on tag error | P-control on tag height |
| `TAG_LOST` | no fresh tag detection | level (0) | `hover_thrust` (hold altitude) |

The attitude inner loop (desired tilt → body rate, see §5) is active in
**every** flying state: with `IGNORE_ATTITUDE` set, a zero body-rate command
means "keep the current tilt", not "fly level" — so "hold" states command
*level attitude*, never zero rates.

Safety properties baked in:

- Disarming or leaving GUIDED_NOGPS from **any** state drops back to
  `WAIT_ARM` immediately (rates zeroed, thrust neutral).
- The vehicle never climbs blindly: `CLIMB` is time-boxed
  (`climb_settle_time`), and afterwards climbing only happens under a fresh
  tag detection. No tag → `TAG_LOST` → altitude hold.
- `ALIGNED` has hysteresis (`align_hysteresis`) so it doesn't chatter at the
  tolerance boundary, and requires a dwell time (`align_dwell_time`) so a
  single lucky sample can't declare success.

Current state is published latched on `ibvs/state` and logged on every
transition.

## 5. Control laws

With tag position `(t_x, t_y, t_z)` in body FLU and targets
`(target_x, target_y, target_z)`:

**X-Y — a cascade, not a direct rate law.** A body-rate command directly
proportional to position error is **unstable**: the commanded rate integrates
into an ever-growing tilt with no attitude feedback (flight-tested in SITL —
the vehicle oscillated laterally, flipped, and ArduPilot's crash check
disarmed it). The stable structure is two nested proportional loops that
still output body rates:

```
# outer loop: PID on the tag position error -> desired tilt (small!)
desired_pitch = PID_x( error=t_x - target_x, error_dot=-v_x )   # clamped ±max_tilt
desired_roll  = PID_y( error=target_y - t_y, error_dot=+v_y )   # clamped ±max_tilt

# inner loop: attitude error -> body rate (runs in ALL flying states)
body_rate.y = clamp( kp_att * (desired_pitch - pitch), ±max_body_rate )
body_rate.x = clamp( kp_att * (desired_roll  - roll),  ±max_body_rate )
```

Each `PID` is a standard `kp·e + ki·∫e + kd·ė` with output clamping and
integral anti-windup (`i_max` bounds the integral *contribution*). **I and D
gains default to 0** — the shipped tuning is pure P. The derivative input is
the body-frame velocity from odometry (for a stationary tag,
`d(error)/dt = ∓velocity`) rather than a numerical derivative of the
detection — same signal, far less noise. Setting `kd` > 0 reproduces the
velocity-damping cascade (`kd ≈ 0.25` was flight-tested and smooths the
approach).

`v_x, v_y` are body-frame velocities and `roll, pitch` the current attitude,
both from `mavros/local_position/odom`. Sign conventions (body FLU, ROS euler
angles): **+pitch = nose down = +x acceleration; +roll = right side down =
−y acceleration** — hence the error signs above (fly *toward* the tag:
`t_x > target` → pitch down; `t_y > target` → roll left).

**Z (ALIGN / ALIGNED):** regulate the tag's height above the vehicle to the
standoff `target_z` with the same PID structure:

```
thrust = hover_thrust + PID_z( error=t_z - target_z, error_dot=-v_z )
# PID_z output clamped to [thrust_min - 0.5, thrust_max - 0.5]
```

Tag well above the standoff → thrust > 0.5 → climb toward it. At the standoff
→ 0.5 → hover. Slightly past it → gentle descent (bounded by `thrust_min`).

**Z (CLIMB):** constant `climb_thrust` — this doubles as the takeoff.

## 6. Nodes, topics & parameters

### `ibvs_controller.py`

| Interface | Name | Type | Notes |
|---|---|---|---|
| sub | `mavros/state` | `mavros_msgs/State` | armed flag + flight mode |
| sub | `ibvs/tag_pose` | `geometry_msgs/PoseStamped` | tag position, body FLU |
| sub | `mavros/local_position/odom` | `nav_msgs/Odometry` | attitude + body velocity for the cascade |
| pub | `mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | at `control_rate` |
| pub | `ibvs/state` | `std_msgs/String` | latched, on transitions |

Parameters (all private, loaded from
[`config/ibvs_params.yaml`](config/ibvs_params.yaml)):

| Param | Default | Meaning |
|---|---|---|
| `control_rate` | `20.0` | setpoint publish rate [Hz] |
| `hover_thrust` | `0.5` | zero-climb-rate command |
| `climb_thrust` | `0.6` | climb command during `CLIMB` (**must be > 0.5 to take off**) |
| `thrust_min` / `thrust_max` | `0.35` / `0.7` | clamp on the Z command |
| `target_z` | `1.0` | desired tag height above vehicle (standoff) [m] |
| `target_x` / `target_y` | `0.0` | desired lateral tag offset [m] |
| `pid_xy/kp` | `0.15` | desired tilt per meter of tag error [rad/m] |
| `pid_xy/ki` | `0.0` | integral gain (0 = off) |
| `pid_xy/kd` | `0.0` | derivative gain; acts on body velocity (0 = off, `0.25` flight-tested) |
| `pid_xy/i_max` | `0.05` | anti-windup clamp on integral contribution [rad] |
| `pid_z/kp` | `0.2` | climb-rate delta per meter of tag-height error |
| `pid_z/ki` / `pid_z/kd` | `0.0` | integral / derivative gains (0 = off) |
| `pid_z/i_max` | `0.1` | anti-windup clamp on integral contribution |
| `max_tilt` | `0.15` | desired-tilt clamp [rad] (~8.5°) |
| `kp_att` | `1.5` | body rate per rad of attitude error [1/s] |
| `max_body_rate` | `0.35` | roll/pitch rate clamp [rad/s] (~20 °/s) |
| `climb_settle_time` | `3.0` | duration of the `CLIMB` phase [s] |
| `align_tolerance` | `0.15` | X-Y error norm considered aligned [m] |
| `align_dwell_time` | `2.0` | time within tolerance before `ALIGNED` [s] |
| `align_hysteresis` | `1.5` | tolerance multiplier to leave `ALIGNED` |
| `tag_timeout` | `1.0` | detection staleness threshold [s] |

### `mock_ar_tag_publisher.py`

Fakes an AR-tag detector. Reads real odometry, subtracts it from a fixed tag
position in the local/world frame, rotates into body FLU (yaw only), and
publishes the result. Because it reacts to real vehicle motion, the loop
genuinely closes in simulation: as the controller flies toward the tag, the
"detection" error shrinks.

| Interface | Name | Type |
|---|---|---|
| sub | `mavros/local_position/odom` (param `~odom_topic`) | `nav_msgs/Odometry` |
| pub | `ibvs/tag_pose` | `geometry_msgs/PoseStamped` |

| Param | Default | Meaning |
|---|---|---|
| `~publish_rate` | `10.0` | detection rate [Hz] |
| `~tag_world_position` | `[-2.0, 0.0, 1.5]` | tag position, local frame |
| `~odom_topic` | `mavros/local_position/odom` | odometry source |

With the kopterworx default spawn `(0, 0, 0.5)`, the first detection is
exactly the canonical demo scenario: **tag at `(-2, 0, 1)` relative, goal
`(0, 0, 1)`** (aligned in X-Y, 1 m below the tag).

## 7. Building

```bash
cd ~/uav_ws
catkin build ibvs_perching
source devel/setup.bash
```

Dependencies are standard: `rospy`, `mavros_msgs`, `geometry_msgs`,
`nav_msgs`, `std_msgs`, `tf` (runtime, mock only).

## 8. Running the simulation demo

```bash
cd ~/uav_ws/src/ibvs_perching/startup/sim_ibvs
./start.sh
```

This starts a tmuxinator session (same pattern as `perching_uav/startup/sim`)
with windows:

| Window | Contents |
|---|---|
| `roscore` | roscore, ArduPilot SITL (`sim_vehicle.launch`), MAVROS |
| `gazebo` | kopterworx in Gazebo |
| `visualization` | PlotJuggler / RViz (pre-typed in history, press ↑) |
| `ibvs` | `ibvs_perching.launch` + arm/disarm commands (history) |
| `status` | echoes of `mavros/state`, `ibvs/state`, `ibvs/tag_pose`, `setpoint_raw/attitude`, odometry |

Then, in the `ibvs` window's second pane, press ↑ and run the pre-typed
command:

```bash
rosservice call /$UAV_NAMESPACE/mavros/set_mode 0 GUIDED_NOGPS; sleep 2; \
rosservice call /$UAV_NAMESPACE/mavros/cmd/arming true
```

Expected sequence (watch `ibvs/state` in the `status` window):

1. `WAIT_ARM` → `CLIMB` the moment the FCU reports armed + GUIDED_NOGPS.
2. `CLIMB` (3 s): thrust `0.7` → **vehicle takes off** and climbs.
3. `ALIGN`: pitches toward the tag (`body_rate.y > 0` for the demo scenario),
   regulates height to the 1 m standoff.
4. `ALIGNED`: error < 15 cm for 2 s → rates zeroed, hovering under the tag.

There is intentionally **no** `control_manager` takeoff service call — no
tracker/controller is running; the `CLIMB` state *is* the takeoff.

To land/abort: pane 3 has `arming false` pre-typed, or switch the RC/mode as
usual; the controller falls back to `WAIT_ARM` on any mode change.

## 9. Troubleshooting

**"It does not want to take off"**

- Most likely cause (and the original bug in this package): sending
  `thrust = 0.5` while expecting it to act as motor thrust. In guided modes
  `0.5` means *zero climb rate* → the vehicle stays on the ground. Ensure
  `climb_thrust > 0.5` (default `0.7`).
- The FCU must be **armed** and in **GUIDED_NOGPS** *while* setpoints are
  streaming. The controller streams continuously from startup, so ordering is
  not an issue if it is running before you arm.
- ArduPilot auto-disarms after ~10 s on the ground (`DISARM_DELAY`). If you
  armed, waited, and then expected motion — arm again and let the already
  climbing-commanded controller take over immediately.
- Arming rejected? Check the SITL console (`:ardupilot1` tmux window) for
  pre-arm failures (EKF still initializing right after SITL boot is common —
  wait ~30 s and retry).
- `rostopic hz /$UAV_NAMESPACE/mavros/setpoint_raw/attitude` should show
  ~`control_rate` Hz. If not, the controller node isn't running or the
  namespace is wrong.
- If someone set `GUID_OPTIONS` bit 3 on your FCU, `thrust` becomes raw
  thrust and this controller's Z-channel assumptions no longer hold.

**Stuck in `WAIT_ARM`** — `mavros/state` shows `mode` ≠ `GUIDED_NOGPS` or
`armed: false`. The mode string must match exactly.

**Stuck in `TAG_LOST`** — no fresh `ibvs/tag_pose`. Is the mock (or real
detector) running? Is odometry arriving on `~odom_topic`?

**Drifts away instead of aligning** — gain sign issue: the `kp_pitch` /
`kp_roll` signs were verified against the mock geometry and in SITL, but a
different airframe/firmware convention could flip them; negate the gain.

## 10. Integrating a real AR-tag detector

Launch with the mock disabled and remap your detector's output:

```bash
roslaunch ibvs_perching ibvs_perching.launch use_mock_tag:=false
```

Your detector must publish `geometry_msgs/PoseStamped` on
`/$UAV_NAMESPACE/ibvs/tag_pose` with the tag position **in the body FLU
frame** (camera→body extrinsics applied). Keep the rate ≥ a few Hz;
detections older than `tag_timeout` put the controller into `TAG_LOST`
(altitude hold), which is the intended graceful degradation under occlusion
or detection dropouts.

## 11. Known limitations & future work

- **No yaw control** — `body_rate.z` is always 0; the vehicle keeps its
  arming heading. Fine for a yaw-symmetric approach; add a yaw law if tag
  orientation matters for the perch.
- **P-only control** — no integral action (steady-state offset under wind)
  and no derivative/velocity damping. The firmware's rate loops provide inner
  damping, but aggressive gains will oscillate.
- **Z is regulated on the tag detection only** — with the tag lost, altitude
  is held open-loop (`0.5` climb rate) with no barometer/odometry fallback.
- **No terminal "perch" state** — the machine ends at `ALIGNED` (hover at
  standoff). The actual final approach/contact phase (e.g. handing over to
  the `perching_uav` trajectory pipeline, or a gripper trigger) is the
  natural next state to add.
- Body-rate X-Y control causes lateral drift *while* rotating (rates ≠
  velocities); the P-loop corrects it continuously, but a velocity-based
  outer loop would track faster.
