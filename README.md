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

0. [Quickstart — run everything in Docker](#0-quickstart--run-everything-in-docker)
1. [Motivation & design decisions](#1-motivation--design-decisions)
2. [How ArduPilot interprets the setpoint (READ THIS)](#2-how-ardupilot-interprets-the-setpoint-read-this)
3. [Architecture](#3-architecture)
4. [The state machine](#4-the-state-machine)
5. [Control laws](#5-control-laws)
6. [Nodes, topics & parameters](#6-nodes-topics--parameters)
7. [Building](#7-building)
8. [Running the simulation demo](#8-running-the-simulation-demo)
9. [Troubleshooting (incl. "it does not take off")](#9-troubleshooting)
10. [Integrating a real AR-tag detector](#10-integrating-a-real-ar-tag-detector)
11. [Known limitations & future work](#11-known-limitations--future-work)
12. [Flying for real (`real_world` branch)](#12-flying-for-real-real_world-branch)

---

## 0. Quickstart — run everything in Docker

The fastest way to a flying simulation — no ROS installation, no catkin
workspace, no GitHub account or SSH key. You need a Linux host with
[Docker](https://docs.docker.com/engine/install/ubuntu/); run
`xhost +local:docker` once per login session so Gazebo/RViz can open windows.

```bash
git clone https://github.com/JakobDomislovic/ibvs_perching.git
cd ibvs_perching
./docker/build.sh        # first build takes a while (~20-30 min)
./docker/run.sh          # drops you into startup/sim_ibvs inside the container
./start.sh               # Gazebo + SITL + mavros + IBVS (see section 8)
```

The image bundles the prebuilt LARICS `uav_ros_stack`, the
[`uav_ros_simulation`](https://github.com/larics/uav_ros_simulation) stack
**pinned to a known-good commit**, and this package, already built. By default
`run.sh` mounts your checkout into the container, so you can edit code on the
host and rerun it inside without rebuilding the image.

All the details — GPU/non-GPU variants, all flags, developing inside the
container, updating the pinned simulation — are in
[docker/README.md](docker/README.md) and on the
[documentation site](https://jakobdomislovic.github.io/ibvs_perching/docker.html).

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
   camera/color/image_raw ---> +--------------------------+
   camera/color/camera_info    |      VISION MODULE       |   (aruco_detector.py today;
   (down-facing camera)        | detects "the point" in   |    mock_ar_tag_publisher.py
                               | the image                |    with use_mock_tag:=true;
                               +-----------+--------------+    your own node tomorrow)
                                           | ibvs/target_point (PointStamped:
                                           |  x,y = normalized offset from the
                                           |  image center; z = distance or 0)
                                           v
   mavros/state ------------->  +--------------------------+
   (armed? GUIDED_NOGPS?)       |     ibvs_controller      |--> ibvs/state (String, latched)
   mavros/local_position/odom ->|  modal state machine +   |
   (attitude, velocity, height) |  PID cascade on image    |--> mavros/setpoint_raw/attitude
                                |  point error             |    (AttitudeTarget @ control_rate)
                                +--------------------------+         |
                                                                     v
                                                    MAVROS -> ArduPilot (SET_ATTITUDE_TARGET)
```

Everything runs under the UAV namespace (`$UAV_NAMESPACE`, default `red`), so
topic names above are relative (`/red/ibvs/target_point`, etc).

### The vision module interface

The controller never knows *what* is being tracked — it centers a point in
the camera image. Any node that publishes
`ibvs/target_point` (`geometry_msgs/PointStamped`) is a valid vision module:

| Field | Meaning |
|---|---|
| `point.x` | normalized horizontal offset from the image center: `(u − cx)/fx`, positive **right** |
| `point.y` | normalized vertical offset: `(v − cy)/fy`, positive **down** |
| `point.z` | distance to the target along the optical axis [m], **`0.0` if unknown** |

Rules:

- **Publish only while the target is detected.** Fresh messages are what
  flips the state to `TAG_IN_SIGHT` (and keeps `ALIGN` alive); silence for
  `tag_timeout` means the target is gone.
- With `point.z = 0` the controller *only centers the point laterally* and
  holds altitude (lateral gains are scaled by `depth_guess`). Provide a real
  distance and it will also regulate the vertical standoff (`target_z`).
- The camera is assumed rigidly mounted looking straight **down**, with
  image **right = body forward** (the kopterworx `down_facing_camera`
  mount). A point at image (+x, +y) is (forward, right) of the vehicle.

### The AR-tag simulation setup (`ar_tag` branch)

| Piece | What / where |
|---|---|
| Tag model | `models/ar_tag/` — 20×20 cm ArUco marker (`DICT_4X4_50`, id 0) on a 30×30 cm white plate (5 cm quiet zone), spawned flat on the floor at the **world origin** by `ibvs_perching.launch` |
| Camera | kopterworx down-facing RGB camera, 640×480 @ 30 fps, 80° HFOV, on `camera/color/image_raw` + `camera_info`; detection throttled to 15 Hz |
| Camera mount | `urdf/kopterworx_downcam.urdf.xacro` — a copy of the stock kopterworx xacro with `down_facing_camera` moved to `xyz="0 0 -0.05"` (the stock mount hangs 0.3 m below / 0.2 m left, which would touch the tag at the target altitude and push it out of frame during descent) |
| Vision module | `scripts/aruco_detector.py` — publishes `ibvs/target_point`, see the interface above |
| Mission profile | `ibvs/takeoff` climbs to **2 m** (`takeoff_height`), holds; when `ibvs/state` shows **`TAG_IN_SIGHT`**, `ibvs/start` centers the point in the camera and (because ArUco provides depth) descends to the standoff above the tag (`target_z`, negative = tag below) |
| Spawn point | UAV starts at `(1, 0)`, 1 m from the tag, so the alignment maneuver is visible |
| Signals | `config/plotjuggler_ibvs.xml` — PlotJuggler layout with the commanded body rates, thrust/climb-rate, tag error and altitude (pre-typed in the `visualization` window) |

**Detection floor:** the full marker must be inside the image for ArUco to
detect it. With the 80°(H)/61°(V) FOV and the camera 5 cm below the base,
the 20 cm marker fills the vertical FOV at roughly **0.25 m** altitude, so
the 0.3–0.5 m standoff stays comfortably detectable. If detection flickers,
the controller degrades gracefully (`TAG_LOST` = position hold, re-`ALIGN`
on re-detection). The **descend gate** (`descend_xy_gate`) additionally
refuses to descend while laterally off-center, which is what keeps the tag
inside the shrinking FOV on the way down.

## 4. The state machine

```
          ibvs/takeoff called             settle time          ibvs/start called
          (mode+arm confirmed)            (takeoff done)       & tag fresh
  WAIT_ARM ----------------> CLIMB ----------------> HOVER ----------------> ALIGN
     ^                                                 ^                      |  ^
     |  disarm / mode change (from ANY state)          | ibvs/stop            |  | error >
     |                                                 | (any flying state)   |  | tol*hyst
     |                                                 |                      v  |
     +----------------------------- TAG_LOST <-- tag stale (timeout) --- ALIGNED
                                       |                                      ^
                                       +---------- tag fresh again -----------+
                                              (|err| < tol for dwell time -> ALIGNED)
```

| State | Meaning | desired tilt (X-Y) | thrust (climb rate) |
|---|---|---|---|
| `WAIT_ARM` | waiting for `ibvs/takeoff` (armed+mode alone does nothing) | level (0) | `hover_thrust` (neutral; ignored while disarmed) |
| `CLIMB` | takeoff: climbing to `takeoff_height` | position hold (takeoff point) | `climb_thrust` (**> 0.5 → the vehicle lifts off**) |
| `HOVER` | holding, tag NOT visible — `ibvs/start` would go to `TAG_LOST` | position hold | `hover_thrust` (hold altitude) |
| `TAG_IN_SIGHT` | holding, **detector sees the tag** — call `ibvs/start` now | position hold | `hover_thrust` (hold altitude) |
| `ALIGN` | closed-loop X-Y servoing + Z standoff regulation | PID on tag error | PID on tag height (+ descend gate) |
| `ALIGNED` | error small & settled — hold | PID on tag error | PID on tag height (+ descend gate) |
| `TAG_LOST` | detection lost while servoing | position hold | `hover_thrust` (hold altitude) |

`HOVER` ↔ `TAG_IN_SIGHT` flip automatically with detection; both hold the
same latched position — the label is the operator's cue that `ibvs/start`
will engage immediately.

### Services (all `std_srvs/Trigger`)

| Service | Effect |
|---|---|
| `ibvs/takeoff` | full takeoff: switches to GUIDED_NOGPS, arms, climbs `climb_settle_time` s, then **holds position** in `HOVER` |
| `ibvs/start` | begin servoing toward the tag (`HOVER` → `ALIGN`) |
| `ibvs/stop` | abort servoing, return to `HOVER` (hold in place) |

Arming the FCU yourself (e.g. `mavros/cmd/arming`) no longer triggers a
climb — only `ibvs/takeoff` does. To restore the old one-shot behavior
(takeoff flows straight into alignment) set `~auto_start: true`.

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
| sub | `ibvs/target_point` | `geometry_msgs/PointStamped` | the vision-module interface (see §3) |
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
| `takeoff_height` | `2.0` | `CLIMB` ends when odometry z reaches this [m] |
| `target_z` | `-0.5` | desired target height above vehicle (standoff) [m]; **negative = target below**. Z only moves when the vision module provides depth |
| `depth_guess` | `2.0` | assumed target distance when `point.z = 0` [m]; scales lateral gains only |
| `target_x` / `target_y` | `0.0` | desired lateral offset [m] |
| `pid_xy/kp` | `0.1` | desired tilt per meter of lateral error [rad/m] |
| `pid_xy/ki` | `0.0` | integral gain (0 = off; a small value removes the trim-bias droop) |
| `pid_xy/kd` | `0.15` | derivative gain; acts on body velocity — needed so the descent doesn't outrun the shrinking FOV (flight-tested) |
| `pid_xy/i_max` | `0.05` | anti-windup clamp on integral contribution [rad] |
| `pid_z/kp` | `0.2` | climb-rate delta per meter of tag-height error |
| `pid_z/ki` / `pid_z/kd` | `0.0` | integral / derivative gains (0 = off) |
| `pid_z/i_max` | `0.1` | anti-windup clamp on integral contribution |
| `descend_xy_gate` | `0.25` | landing funnel: descend only while the lateral tag error is inside this radius [m] — descending off-center loses the tag from the shrinking FOV (flight-tested) |
| `max_tilt` | `0.15` | desired-tilt clamp [rad] (~8.5°) |
| `kp_att` | `1.5` | body rate per rad of attitude error [1/s] |
| `max_body_rate` | `0.35` | roll/pitch rate clamp [rad/s] (~20 °/s) |
| `kp_hover` | `0.15` | position-hold P outside ALIGN [rad/m]; `HOVER`/`TAG_LOST` hold a latched position (the takeoff point, or wherever servoing stopped). Level attitude alone drifts away on attitude trim bias (flight-tested ~0.1 m/s) |
| `kv_hover` | `0.25` | velocity damping for the position hold [rad per m/s] |
| `auto_start` | `false` | skip the `ibvs/start` gate: takeoff flows straight into ALIGN |
| `climb_settle_time` | `10.0` | `CLIMB` fallback timeout if `takeoff_height` is never reached [s] |
| `align_tolerance` | `0.15` | X-Y error norm considered aligned [m] |
| `align_dwell_time` | `2.0` | time within tolerance before `ALIGNED` [s] |
| `align_hysteresis` | `1.5` | tolerance multiplier to leave `ALIGNED` |
| `tag_timeout` | `0.5` | detection staleness threshold [s] |

### `aruco_detector.py` (the shipped vision module)

Detects the ArUco marker with `cv2.aruco` and publishes its **image-plane
center** on `ibvs/target_point` (see the interface in §3). `point.x/y` come
straight from the marker's pixel center and the intrinsics (exact even if
`marker_length` is miscalibrated); `point.z` is the depth estimated from the
known marker size.

| Interface | Name | Type |
|---|---|---|
| sub | `camera/color/image_raw` | `sensor_msgs/Image` |
| sub | `camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| pub | `ibvs/target_point` | `geometry_msgs/PointStamped` (vision interface) |
| pub | `ibvs/debug_image` | `sensor_msgs/Image` (detections drawn; only rendered when subscribed — `rqt_image_view` is pre-typed in the visualization window) |

| Param | Default | Meaning |
|---|---|---|
| `~marker_id` | `0` | ArUco id to accept |
| `~marker_length` | `0.20` | marker side [m] — only affects the `point.z` depth hint |
| `~process_rate` | `15.0` | detection rate [Hz]; camera frames arriving faster are skipped |
| `~dictionary` | `DICT_4X4_50` | any `cv2.aruco.DICT_*` name |

### `mock_ar_tag_publisher.py` (`use_mock_tag:=true`)

Vision module without a camera: computes what a down-facing camera *would*
see for a target at a fixed world position, from real odometry, and
publishes the same `ibvs/target_point` interface (including the depth). It
only publishes while the target is below the vehicle, so `TAG_IN_SIGHT`
behaves realistically.

| Param | Default | Meaning |
|---|---|---|
| `~publish_rate` | `15.0` | detection rate [Hz] |
| `~tag_world_position` | `[0.0, 0.0, 0.02]` | target position, local frame |
| `~odom_topic` | `mavros/local_position/odom` | odometry source |
| `~min_depth` | `0.1` | minimum distance below the vehicle to count as "in view" [m] |

## 7. Building

*(Using the [Docker quickstart](#0-quickstart--run-everything-in-docker)? Skip
this section — the image builds everything.)*

```bash
cd ~/uav_ws
catkin build ibvs_perching
source devel/setup.bash
```

Dependencies are standard: `rospy`, `mavros_msgs`, `geometry_msgs`,
`nav_msgs`, `std_msgs`, `tf` (runtime, mock only).

## 8. Running the simulation demo

```bash
cd ~/uav_ws/src/ibvs_perching/startup/sim_ibvs   # in the Docker container you are already here
./start.sh
```

This starts a tmuxinator session (same pattern as `perching_uav/startup/sim`)
with windows:

| Window | Contents |
|---|---|
| `roscore` | roscore, ArduPilot SITL (`sim_vehicle.launch`), MAVROS |
| `gazebo` | kopterworx in Gazebo |
| `visualization` | PlotJuggler / RViz (pre-typed in history, press ↑) |
| `ibvs` | `ibvs_perching.launch` + takeoff / start / stop / disarm commands (history) |
| `status` | echoes of `mavros/state`, `ibvs/state`, `ibvs/tag_pose`, `setpoint_raw/attitude`, odometry |

The mission is **two explicit steps** — the pre-typed commands are waiting in
the `ibvs` window's panes (press ↑):

```bash
# STEP 1: takeoff -- sets GUIDED_NOGPS, arms, climbs ~3 s, then HOLDS in place
rosservice call /$UAV_NAMESPACE/ibvs/takeoff

# STEP 2 (whenever you're ready): fly to the tag
rosservice call /$UAV_NAMESPACE/ibvs/start

# optional: abort alignment, hold current position
rosservice call /$UAV_NAMESPACE/ibvs/stop
```

Expected sequence (watch `ibvs/state` in the `status` window, and the
camera in `rqt_image_view` on `ibvs/debug_image`):

1. `ibvs/takeoff` → `WAIT_ARM` → `CLIMB` (thrust `0.6` until 2 m) →
   **holding position at 2 m**. Nothing else happens until you say so.
2. Watch `ibvs/state`: when the down-facing camera picks up the marker it
   flips `HOVER` → **`TAG_IN_SIGHT`** — that's your cue.
3. `ibvs/start` → `ALIGN`: first centers the tag laterally (the descend
   gate blocks descent while off-center), then descends onto it,
   regulating the standoff to `target_z` above the tag.
4. `ALIGNED`: error < 15 cm for 2 s → holding centered above the tag.

There is intentionally **no** `control_manager` takeoff service call — no
tracker/controller is running; the `CLIMB` state *is* the takeoff. Note that
arming via `mavros/cmd/arming` alone does **not** climb anymore: the
`ibvs/takeoff` service is the only way to lift off (it arms for you).

To land/abort: `ibvs/stop` holds in place; `arming false` (pre-typed) kills
the motors; any RC/mode change drops the controller back to `WAIT_ARM`.

## 9. Troubleshooting

**"It does not want to take off"**

- Did you call `ibvs/takeoff`? Arming by hand no longer climbs — the takeoff
  service is the only trigger (it also arms for you).
- Most likely cause (and the original bug in this package): sending
  `thrust = 0.5` while expecting it to act as motor thrust. In guided modes
  `0.5` means *zero climb rate* → the vehicle stays on the ground. Ensure
  `climb_thrust > 0.5` (default `0.6`).
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

## 12. Flying for real (`real_world` branch)

```bash
cd ~/uav_ws/src/ibvs_perching/startup/real_world
./start.sh                    # or ./start.sh my_aircraft_setup.sh
```

Most of what used to come from `uav_ros_stack` is now package-local:

| Was (uav_ros_general) | Now (ibvs_perching) |
|---|---|
| `apm2.launch` + `mavros_node.launch` | `launch/mavros_apm.launch` + `config/apm_config.yaml` |
| `waitForRos`/`waitForMavros`/`waitForSysStatus` shell helpers | `startup/real_world/shell_helpers.sh` |

The **joystick stays the standard one**: the session launches
`uav_ros_general rc_to_joy.launch mapping_file:=$RC_MAPPING`, exactly like
`perching_uav/startup/rw` (`scripts/rc_to_joy.py` is a drop-in Python port
of that node, kept only as a fallback for an aircraft without the stack).
Per-aircraft settings (FCU serial port, RC channel mapping) live in
`startup/real_world/rw_setup.sh` and `custom_config/rc_mapping.yaml`.

**The engagement flow — no `position_hold` service.** The controller runs
with `engage_on_target: true` (`custom_config/ibvs_params_rw.yaml`):

1. The safety pilot takes off **manually** (STABILIZE) and flies to the
   area. The controller sits in `WAIT_ARM`, streaming (ignored) setpoints.
2. Press the **IBVS button** — `ibvs/start` (`i` on the sim keyboard
   joystick; a real joystick button later). The next fresh point on
   `ibvs/target_point` — a tag detection, or any point you choose to
   publish — engages: the controller switches the FCU to `GUIDED_NOGPS`
   itself and goes straight to `ALIGN` (`CLIMB` is skipped, the vehicle is
   already airborne). If the point goes stale within `tag_timeout`, it
   simply **holds position** (`TAG_LOST`) — button + one empty point is
   effectively position hold. (Set `engage_needs_start: false` to skip
   the button and engage on the very first point.)
3. The safety pilot can **always** take back control with the RC mode
   switch. The software mode switch is one-shot: after a takeover the
   controller never re-takes the mode on its own. Flip the RC switch back
   to `GUIDED_NOGPS` to re-engage, or press the button again to let the
   next target point engage. `ibvs/stop` drops servoing back to a hover.

Before the first flight check `GUID_OPTIONS = 0` on the FCU (the `ibvs`
tmux pane sets it): with `GUID_OPTIONS = 8` the thrust field is raw thrust
instead of climb rate and the controller **flies away** (section 2).

**Rehearsing this flow in simulation.** `scripts/keyboard_rc.py` is a
keyboard "RC transmitter" for SITL: it flies the vehicle through
`mavros/rc/override`, so ArduPilot sees real RC input (and reports it on
`mavros/rc/in`, which is why the same `rc_to_joy` bridge works in sim).
On startup it sets `SYSID_MYGCS = 1` on the FCU — without that ArduPilot
only accepts overrides from MAVProxy (sysid 255) and silently ignores
mavros, i.e. the keyboard would do nothing.

In the sim session (`startup/sim_ibvs`) the `joystick` window has it
pre-typed (press ↑), and the sim config already has `engage_on_target` +
`engage_needs_start` enabled — no relaunch or special config needed:

`o` arm → `2` ALT_HOLD → hold `w` to climb → arrows (roll/pitch) and
`a`/`d` (yaw) to fly over the tag → **`i` (start IBVS)** → the next
detection engages `GUIDED_NOGPS` by itself. Press `2` to "take over" like
the safety pilot (the controller must not steal the mode back), `g` to
hand control back (or `i` again to re-arm engagement), `k` to stop
servoing, `q` to quit (releases all overrides). Sticks spring back to
center when a key is released; fly in ALT_HOLD, not STABILIZE — centered
throttle holds altitude, which is what keyboard flying needs.
