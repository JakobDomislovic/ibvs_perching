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
| Separate mock detector node | The controller only sees a `PointStamped` on `ibvs/target_point`. Swapping the mock for a real detector requires zero controller changes. |

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
   (down-facing camera,        |      VISION MODULE       |   (aruco_detector.py today;
    no calibration needed)     | detects "the point" in   |    mock_ar_tag_publisher.py
                               | the image                |    with use_mock_tag:=true;
                               +-----------+--------------+    your own node tomorrow)
                                           | ibvs/target_point (PointStamped:
                                           |  x,y = target pixel u,v,
                                           |  0..image_width/height; no depth)
                                           v
   mavros/state ------------->  +--------------------------+
   (armed? GUIDED_NOGPS?)       |     ibvs_controller      |--> ibvs/state (String, latched)
   mavros/local_position/odom ->|  modal state machine +   |
   (attitude, velocity, height) |  PID cascade on pixel    |--> mavros/setpoint_raw/attitude
                                |  error (frame-fraction)  |    (AttitudeTarget @ control_rate)
                                +--------------------------+         |
                                                                     v
                                                    MAVROS -> ArduPilot (SET_ATTITUDE_TARGET)
```

Everything runs under the UAV namespace (`$UAV_NAMESPACE`, default `red`), so
topic names above are relative (`/red/ibvs/target_point`, etc).

### The vision module interface

The controller never knows *what* is being tracked — it centers a point in
the camera image. Any node that publishes
`ibvs/target_point` (`geometry_msgs/PointStamped`) is a valid vision module,
and **needs no camera calibration** — a plain object detector that only
knows a pixel center is enough:

| Field | Meaning |
|---|---|
| `point.x` | target pixel column (`u`), `0 .. image_width` |
| `point.y` | target pixel row (`v`), `0 .. image_height` |
| `point.z` | unused, always `0.0` — **there is no depth** |

Rules:

- **Publish only while the target is detected.** Fresh messages are what
  flips the state to `TAG_IN_SIGHT` (and keeps `ALIGN` alive); silence for
  `tag_timeout` means the target is gone.
- The controller's `~image_width`/`~image_height` must be set to the SAME
  resolution the vision module is using (set once, together, via
  `launch/ibvs_perching.launch`'s `image_width`/`image_height` args) so it
  can find the frame center.
- **No depth means no metric standoff.** The controller cannot know how far
  away the target is. **For now** `ALIGN`/`ALIGNED` only center the point
  laterally and hold `hover_thrust` — no descent at all yet, see §5.
- The camera is assumed rigidly mounted looking straight **down**, with
  image **right = body forward** (the kopterworx `down_facing_camera`
  mount). A point right-of-center in the image is to the right of the
  vehicle.

### The AR-tag simulation setup (`ar_tag` branch)

| Piece | What / where |
|---|---|
| Tag model | `models/ar_tag/` — 20×20 cm ArUco marker (`DICT_4X4_50`, id 0) on a 30×30 cm white plate (5 cm quiet zone), spawned flat on the floor at the **world origin** by `ibvs_perching.launch` |
| Camera | kopterworx down-facing RGB camera, 640×480 @ 30 fps, 80° HFOV, on `camera/color/image_raw` (no `camera_info`/calibration needed — pixel-only interface); detection throttled to 15 Hz |
| Camera mount | `urdf/kopterworx_downcam.urdf.xacro` — a copy of the stock kopterworx xacro with `down_facing_camera` moved to `xyz="0 0 -0.05"` (the stock mount hangs 0.3 m below / 0.2 m left, which would touch the tag at low altitude and push it out of frame) |
| Vision module | `scripts/aruco_detector.py` — publishes `ibvs/target_point`, see the interface above |
| Mission profile | `ibvs/takeoff` climbs to **2 m** (`takeoff_height`), holds; when `ibvs/state` shows **`TAG_IN_SIGHT`**, `ibvs/start` centers the point in the camera and holds **whatever height the vehicle was at when `ibvs/start` was called** (usually ≈2 m, but not tied to `takeoff_height` — see §5) — **no descent yet** (§11), the mission currently ends at centered-and-hovering |
| Spawn point | UAV starts at `(1, 0)`, 1 m from the tag, so the alignment maneuver is visible |
| Signals | `config/plotjuggler_ibvs.xml` — PlotJuggler layout with the commanded body rates, thrust/climb-rate, tag error and altitude (pre-typed in the `visualization` window) |

**Detection floor:** the full marker must be inside the image for ArUco to
detect it. With the 80°(H)/61°(V) FOV and the camera 5 cm below the base,
the 20 cm marker fills the vertical FOV at roughly **0.25 m** altitude —
not currently relevant at the 2 m takeoff height since the controller
doesn't descend yet (§11), but worth knowing before re-enabling descent. If
detection flickers, the controller degrades gracefully (`TAG_LOST` =
position hold, re-`ALIGN` on re-detection).

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
| `ALIGN` | closed-loop X-Y servoing; Z holds altitude (no descent yet) | PID on pixel error | regulated toward `hold_height` |
| `ALIGNED` | error small & settled — hold | PID on pixel error | regulated toward `hold_height` |
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

No depth is available (see §3) — the vision module gives only a pixel
center. `target_callback` converts it to `(t_x, t_y)`, the target's offset
from the frame center **as a fraction of the half-frame** (`0` = centered,
`±1` = the frame edge), against targets `(target_x, target_y)` (default
`0, 0`):

```python
norm_x = (point.x - image_width  / 2) / (image_width  / 2)
norm_y = (point.y - image_height / 2) / (image_height / 2)
t_x, t_y = norm_x, -norm_y   # body FLU sign convention
```

**X-Y — a cascade, not a direct rate law.** A body-rate command directly
proportional to position error is **unstable**: the commanded rate integrates
into an ever-growing tilt with no attitude feedback (flight-tested in SITL —
the vehicle oscillated laterally, flipped, and ArduPilot's crash check
disarmed it). The stable structure is two nested proportional loops that
still output body rates — structurally unchanged from before, just fed a
frame-fraction error instead of a metric one:

```
# outer loop: PID on the frame-fraction error -> desired tilt (small!)
desired_pitch = PID_x( error=t_x - target_x, error_dot=-v_x )   # clamped ±max_tilt
desired_roll  = PID_y( error=target_y - t_y, error_dot=+v_y )   # clamped ±max_tilt

# inner loop: attitude error -> body rate (runs in ALL flying states)
body_rate.y = clamp( kp_att * (desired_pitch - pitch), ±max_body_rate )
body_rate.x = clamp( kp_att * (desired_roll  - roll),  ±max_body_rate )
```

Each `PID` is a standard `kp·e + ki·∫e + kd·ė` with output clamping and
integral anti-windup (`i_max` bounds the integral *contribution*). **I and D
gains default to 0** — the shipped tuning is pure P. The derivative input is
the body-frame velocity from odometry, used as a damping proxy — for a
*metric* error this was an **exact** derivative (`d(error)/dt = ∓velocity`
for a stationary target); for the frame-fraction error it is only an
**approximation**, since the same velocity produces a bigger apparent pixel
motion the closer the vehicle is to the target — a scaling that came for
free with a real depth measurement and cannot be corrected without one.
Setting `kd` > 0 still meaningfully damps the approach (`kd ≈ 0.15` is the
shipped starting point) but expect to retune it.

`v_x, v_y` are body-frame velocities and `roll, pitch` the current attitude,
both from `mavros/local_position/odom`. Sign conventions (body FLU, ROS euler
angles): **+pitch = nose down = +x acceleration; +roll = right side down =
−y acceleration** — hence the error signs above (fly *toward* the target:
`t_x > target` → pitch down; `t_y > target` → roll left).

**Z (ALIGN / ALIGNED) — holds altitude, for now.** With no depth there is
nothing to regulate a standoff against, and no depth-free descent strategy
is enabled yet either. Instead of trusting an open-loop `hover_thrust` to
hold still, the controller latches the odometry height **at the moment
servoing starts** (entering `ALIGN` — i.e. wherever the vehicle actually
was when `ibvs/start` was called, not `takeoff_height`) and regulates
toward it, same P + velocity-damping structure as the X-Y position hold
above:

```python
# hold_height latched once, in transition(), on entering ALIGN:
#   hold_height = odom.z
z_err = hold_height - odom.z
thrust = hover_thrust + clamp(kp_alt_hold * z_err - kv_alt_hold * v_z,
                              thrust_min - hover_thrust, thrust_max - hover_thrust)
```

This matters most for the `engage_on_target` real-world flow (§12), where
`CLIMB` never runs at all — the vehicle is already airborne wherever the
safety pilot flew it, and `hold_height` is the only thing anchoring
altitude once `ALIGN` takes over. In the sim service flow it also guards
against drift accumulated during an arbitrarily long wait in `HOVER`
before `ibvs/start` is finally called.

Centering the target laterally does not by itself bring the vehicle any
closer to it — `ALIGN`/`ALIGNED` currently mean "centered and holding,"
not "descending toward the target." An earlier iteration of this
controller had a depth-free open-loop descent ramp here (thrust stepping
down from `hover_thrust` while centered, gated by lateral error and a
minimum-altitude safety floor) — removed for now, see §11 for the idea if
it's worth reviving.

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
| `image_width` / `image_height` | `640` / `480` | camera resolution — the only thing the controller needs to know about the camera. Set together with the vision module via `launch/ibvs_perching.launch`'s args, not this file |
| `hover_thrust` | `0.5` | zero-climb-rate command |
| `climb_thrust` | `0.6` | climb command during `CLIMB` (**must be > 0.5 to take off**) |
| `thrust_min` / `thrust_max` | `0.35` / `0.7` | clamp on the Z command |
| `takeoff_height` | `2.0` | `CLIMB` ends when odometry z reaches this [m] |
| `kp_alt_hold` | `0.3` | altitude-hold P outside CLIMB [thrust/m]; `ALIGN`/`ALIGNED` hold `hold_height`, the odometry height latched when servoing started (§5) — **not** `takeoff_height` |
| `kv_alt_hold` | `0.2` | climb-rate damping for the altitude hold [thrust per m/s] |
| `target_x` / `target_y` | `0.0` | desired lateral offset, frame-fraction (0 = centered) |
| `pid_xy/kp` | `0.1` | desired tilt per unit of frame-fraction error [rad] — retune from the metric-era default, units changed |
| `pid_xy/ki` | `0.0` | integral gain (0 = off; a small value removes the trim-bias droop) |
| `pid_xy/kd` | `0.15` | derivative gain; acts on body velocity as an approximate damping proxy (no longer an exact derivative without depth, see §5) |
| `pid_xy/i_max` | `0.05` | anti-windup clamp on integral contribution [rad] |
| `max_tilt` | `0.15` | desired-tilt clamp [rad] (~8.5°) |
| `kp_att` | `1.5` | body rate per rad of attitude error [1/s] |
| `max_body_rate` | `0.35` | roll/pitch rate clamp [rad/s] (~20 °/s) |
| `kp_hover` | `0.15` | position-hold P outside ALIGN [rad/m]; `HOVER`/`TAG_LOST` hold a latched position (the takeoff point, or wherever servoing stopped). Level attitude alone drifts away on attitude trim bias (flight-tested ~0.1 m/s) |
| `kv_hover` | `0.25` | velocity damping for the position hold [rad per m/s] |
| `auto_start` | `false` | skip the `ibvs/start` gate: takeoff flows straight into ALIGN |
| `climb_settle_time` | `10.0` | `CLIMB` fallback timeout if `takeoff_height` is never reached [s] |
| `align_tolerance` | `0.08` | X-Y error norm (frame-fraction) considered aligned — retune from the metric-era default, units changed |
| `align_dwell_time` | `2.0` | time within tolerance before `ALIGNED` [s] |
| `align_hysteresis` | `1.5` | tolerance multiplier to leave `ALIGNED` |
| `tag_timeout` | `0.5` | detection staleness threshold [s] |

### `aruco_detector.py` (the shipped vision module)

Detects the ArUco marker with `cv2.aruco` and publishes its **pixel-plane
center** on `ibvs/target_point` (see the interface in §3) — no camera
intrinsics or calibration used at all, `point.x/y` are exactly the marker's
detected pixel `(u, v)`. `point.z` is always `0.0`: `marker_length` is kept
only because ArUco needs *some* marker size to run detection, but no depth
is estimated or published.

| Interface | Name | Type |
|---|---|---|
| sub | `camera/color/image_raw` | `sensor_msgs/Image` |
| pub | `ibvs/target_point` | `geometry_msgs/PointStamped` (vision interface) |
| pub | `ibvs/debug_image` | `sensor_msgs/Image` (detections drawn; only rendered when subscribed — `rqt_image_view` is pre-typed in the visualization window) |

| Param | Default | Meaning |
|---|---|---|
| `~marker_id` | `0` | ArUco id to accept |
| `~process_rate` | `15.0` | detection rate [Hz]; camera frames arriving faster are skipped |
| `~dictionary` | `DICT_4X4_50` | any `cv2.aruco.DICT_*` name |

### `mock_ar_tag_publisher.py` (`use_mock_tag:=true`)

Vision module without a camera: computes what a down-facing camera *would*
see for a target at a fixed world position, from real odometry plus an
assumed `~horizontal_fov`, and publishes the same pixel-only
`ibvs/target_point` interface. The odometry-derived depth is used
internally only to project the target and to decide whether it's "in
view" — it is never published, matching what a real pixel-only detector
would (and wouldn't) know.

| Param | Default | Meaning |
|---|---|---|
| `~publish_rate` | `15.0` | detection rate [Hz] |
| `~tag_world_position` | `[0.0, 0.0, 0.02]` | target position, local frame |
| `~odom_topic` | `mavros/local_position/odom` | odometry source |
| `~min_depth` | `0.1` | minimum distance below the vehicle to count as "in view" [m] |
| `~image_width` / `~image_height` | `640` / `480` | must match the controller's (set via the launch file) |
| `~horizontal_fov` | `1.3962634` | assumed camera FOV [rad], used only to fake a believable pixel position (default: kopterworx down camera, 80°) |

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
| `status` | echoes of `mavros/state`, `ibvs/state`, `ibvs/target_point`, `setpoint_raw/attitude`, odometry |

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
3. `ibvs/start` → `ALIGN`: centers the tag laterally and holds whatever
   height the vehicle is at **right now** (`hold_height`, latched at this
   exact moment — usually ≈2 m here, but it's this event that anchors it,
   not `takeoff_height`) — **no descent yet** (§11): no depth means the
   controller cannot know how far above the tag it is, and no depth-free
   descent strategy is enabled for now either.
4. `ALIGNED`: error norm < `align_tolerance` for 2 s → holding centered at
   `hold_height`.

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

**Stuck in `TAG_LOST`** — no fresh `ibvs/target_point`. Is the mock (or real
detector) running? Is odometry arriving on `~odom_topic`?

**Drifts away instead of aligning** — gain sign issue: the `kp_pitch` /
`kp_roll` signs were verified against the mock geometry and in SITL, but a
different airframe/firmware convention could flip them; negate the gain.

## 10. Integrating a real AR-tag detector

Launch with the mock disabled and remap your detector's output:

```bash
roslaunch ibvs_perching ibvs_perching.launch use_mock_tag:=false
```

Your detector must publish `geometry_msgs/PointStamped` on
`/$UAV_NAMESPACE/ibvs/target_point` with the target's **pixel** coordinates
(see the interface in §3) — no camera calibration or depth needed, just the
detected center and matching `~image_width`/`~image_height` on the
controller. Keep the rate ≥ a few Hz; detections older than `tag_timeout`
put the controller into `TAG_LOST` (altitude hold), which is the intended
graceful degradation under occlusion or detection dropouts.

## 11. Known limitations & future work

- **No yaw control** — `body_rate.z` is always 0; the vehicle keeps its
  arming heading. Fine for a yaw-symmetric approach; add a yaw law if tag
  orientation matters for the perch.
- **P-only control** — no integral action (steady-state offset under wind)
  and no derivative/velocity damping. The firmware's rate loops provide inner
  damping, but aggressive gains will oscillate.
- **No descent at all, for now** — `ALIGN`/`ALIGNED` only center the target
  laterally and hold `hold_height` (the height when servoing started, §5);
  centering it does not bring the vehicle any closer. This was a
  deliberate simplification, not a fundamental limit: a previous iteration
  had a depth-free open-loop
  descent ramp (thrust stepping down from `hover_thrust` while laterally
  centered, gated by lateral error and a minimum-odometry-altitude safety
  floor — see git history / §5 for the idea) that can be reintroduced. A
  vision module that *can* estimate distance (e.g. from a known object
  size) could instead reintroduce a depth field and a proper standoff PID.
- **No terminal "perch" state** — the machine ends at `ALIGNED` (centered,
  hovering). The actual descent, final approach/contact phase (e.g. handing
  over to the `perching_uav` trajectory pipeline, or a gripper trigger) is
  the natural next state to add.
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
   already airborne). Altitude is held at exactly **wherever the vehicle
   was flying at this moment** (`hold_height`, §5) — this flow never uses
   `takeoff_height` at all. If the point goes stale within `tag_timeout`, it
   simply **holds position** (`TAG_LOST`) — button + one point at the
   frame center (`image_width/2, image_height/2` — **not** `(0,0)`, which
   is the top-left pixel corner, not the center) is effectively position
   hold. (Set `engage_needs_start: false` to skip the button and engage on
   the very first point.)
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
