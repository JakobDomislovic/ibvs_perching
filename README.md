# ibvs_perching (PiOS branch)

**Image-Based Visual Servoing (IBVS) for perching and landing, driven by
off-board detection streamed from the PiOS companion over UDP.**

This branch is the flight-only stack. The PiOS companion computer detects the
target (a branch for perching, an ArUco tag for landing) and streams its pixel
position over UDP; `udp_target_receiver.py` republishes that as
`ibvs/target_point`, and `ibvs_controller.py` flies ArduPilot **directly**
through `mavros/setpoint_raw/attitude`.

There is **no** Gazebo/SITL simulation, no on-board ArUco detection, no
RealSense, no OptiTrack and no `uav_ros_stack` dependency — the only external
ROS package needed is `mavros`. (The simulation, docker image, URDF/models and
on-board detector live on the other branches.)

**No camera calibration anywhere.** The controller normalizes the incoming
pixel position by each axis' half-dimension, so the error is ±1.0 at that
axis' frame edge. No focal length, no principal point — which also means the
gains and tolerances survive a resolution change.

---

## Table of contents

1. [Quickstart](#1-quickstart)
2. [How ArduPilot interprets the setpoint (READ THIS)](#2-how-ardupilot-interprets-the-setpoint-read-this)
3. [Architecture](#3-architecture)
4. [The state machine](#4-the-state-machine)
5. [Control laws](#5-control-laws)
6. [Nodes, topics & parameters](#6-nodes-topics--parameters)
7. [Building](#7-building)
8. [Checking the signs without flying](#8-checking-the-signs-without-flying)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Quickstart

```bash
cd startup/real_world
$EDITOR start_udp.sh     # the CONFIG block at the top is the only thing to edit
./start_udp.sh           # roscore + mavros + UDP receiver + IBVS, in tmux
```

`start_udp.sh` is the single source of truth for every knob that is not a
control gain:

| Setting | Meaning |
|---|---|
| `FCU_URL` | mavros serial link to the flight controller |
| `MISSION_MODE` | `perch` (up cam, climb) or `land` (down cam, descend + disarm) |
| `UAV_NAMESPACE` | ROS namespace for mavros + ibvs |
| `BIND_PORT` | UDP port the PiOS detector sends to |
| `IMAGE_WIDTH` / `IMAGE_HEIGHT` | resolution the detector reports `px`/`py` in |

Control gains and the alignment thresholds live in
[`startup/real_world/custom_config/ibvs_params_rw.yaml`](startup/real_world/custom_config/ibvs_params_rw.yaml).

> `IMAGE_WIDTH`/`IMAGE_HEIGHT` **must** match the resolution the PiOS detector
> reports in. If it downscales before detecting, use the **downscaled** size,
> not the capture size. Verify by centring the target and reading the incoming
> `px`/`py` on `ibvs/target_point`.

### The flight flow

1. The safety pilot takes off and flies **manually** (STABILIZE).
2. The controller streams setpoints continuously, but ArduPilot only acts on
   them in **GUIDED_NOGPS** — and the **pilot** selects that on the RC mode
   switch. This node never changes the flight mode by itself.
3. With `engage_on_target: true` (the configured value) servoing begins as
   soon as the vehicle is armed, in GUIDED_NOGPS, and a fresh detection
   arrives.
4. Flipping the RC mode switch away hands control back **instantly**, from any
   state. `ibvs/stop` also disables servoing; `ibvs/start` re-enables it.

---

## 2. How ArduPilot interprets the setpoint (READ THIS)

`mavros_msgs/AttitudeTarget` is sent as MAVLink `SET_ATTITUDE_TARGET`.

**The `thrust` field is NOT motor thrust.** In GUIDED / GUIDED_NOGPS,
ArduCopter interprets `thrust` as a **normalized climb-rate command** — unless
`GUID_OPTIONS` bit 3 ("SetAttitudeTarget interprets Thrust as Thrust") is set:

| `thrust` value | Commanded vertical motion |
|---|---|
| `0.0` | descend at maximum rate (`PILOT_SPEED_DN`) |
| `0.5` | **zero climb rate — hold altitude ("hover")** |
| `1.0` | climb at maximum rate (`PILOT_SPEED_UP`) |

> **⚠ CRITICAL — `GUID_OPTIONS` must be 0.** The LARICS `identity.parm`
> (flashed on the real vehicles) sets **`GUID_OPTIONS = 8`**, i.e.
> thrust-as-raw-thrust, because the `uav_ros_stack` MPC computes true thrust
> through its thrust model. With that setting this controller's commands are
> all above the vehicle's hover throttle (`MOT_THST_HOVER ≈ 0.29`) and it
> **flies away at a constant climb** (flight-tested: ~5.4 m/s, straight past
> 300 m). The `session_udp.yml` startup therefore runs `setParam GUID_OPTIONS 0`
> before launching the controller — set it back for the normal MPC stack.

Two command modes for the lateral axes, selected by `~command_mode`:

- **`attitude`** (configured): publish the target **attitude quaternion** and
  let ArduPilot close the attitude loop at 400 Hz with its tuned `ATC_*`
  gains. `type_mask = 7` (ignore body rates).
- **`rate`**: publish a body rate from our own `kp_att` inner loop.
  `type_mask = 128` (`IGNORE_ATTITUDE`). Note that with `IGNORE_ATTITUDE` a
  zero body-rate command means "keep the current tilt", not "fly level".

---

## 3. Architecture

```
  PiOS companion                    flight computer (this package)
 ┌────────────────┐               ┌──────────────────────────────────────┐
 │ USB camera     │               │  udp_target_receiver.py              │
 │   ↓            │   UDP/JSON    │    (bind_port, JSON -> PointStamped) │
 │ detector       │ ────────────► │            ↓                         │
 │ (branch/ArUco) │  {"px","py"}  │      ibvs/target_point               │
 └────────────────┘               │            ↓                         │
                                  │  ibvs_controller.py                  │
                                  │    normalize -> PID -> tilt          │
                                  │            ↓                         │
                                  │  mavros/setpoint_raw/attitude        │
                                  └──────────────┬───────────────────────┘
                                                 ↓
                                          mavros → ArduPilot
```

### The vision module interface

The controller is agnostic to **what** is being tracked. Anything that
publishes `ibvs/target_point` (`geometry_msgs/PointStamped`) is a valid vision
module:

| Field | Meaning |
|---|---|
| `point.x` | horizontal **pixel position** of the detection, positive **RIGHT** from the image origin (top-left) |
| `point.y` | vertical **pixel position** of the detection, positive **DOWN** from the image origin (top-left) |
| `point.z` | **ignored** — there is no range sensor; the vertical axis is open-rate |

`udp_target_receiver.py` publishes the detected **point**, not an error: it
forwards what the detector saw and does no geometry. The controller owns the
setpoint (`~target_x` / `~target_y`, a frame-fraction offset where 0.0 =
centred) and forms the error itself as `detection - target`.

### UDP wire format

One JSON object per datagram, on `BIND_PORT`:

```json
{"px": 640, "py": 360}
```

`error_x` / `error_y` are also accepted for detectors that already emit
centre-relative values — they are forwarded unchanged, so set the controller's
`~target_x` / `~target_y` to 0 in that case, or the centre is subtracted
twice.

### Image → body signs

`image_x_sign` / `image_y_sign` mirror the error about the image centre, i.e.
they set which way the vehicle leans for a given detection offset. **The rule:
the commanded lean must point toward where the target physically is, not
toward where it appears in the image.**

| Mission | Camera | `image_x_sign` | `image_y_sign` |
|---|---|---|---|
| `land` | down-facing | `+1.0` | `+1.0` |
| `perch` | up-facing | `-1.0` | `+1.0` |

The same physical camera turned to face up inverts the **x** axis only. Check
any change with `./sim_tilt.py --sweep` (§8) before flying.

---

## 4. The state machine

```
          ibvs/takeoff called             settle time          engaged
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
| `WAIT_ARM` | disarmed or not in GUIDED_NOGPS — idle | level (0) | `hover_thrust` (neutral) |
| `CLIMB` | `ibvs/takeoff` path only: climbing to `takeoff_height` | level | `climb_thrust` (**> 0.5 → lifts off**) |
| `HOVER` | holding, target NOT visible | level | `hover_thrust` |
| `TAG_IN_SIGHT` | holding, detector sees the target — engagement is imminent | level | `hover_thrust` |
| `ALIGN` | closed-loop X-Y servoing | PID on the image error | `hover_thrust` until centred |
| `ALIGNED` | error small & settled for `align_dwell_time` | PID on the image error | `perch_climb_thrust` / `land_descend_thrust` |
| `TAG_LOST` | detection went stale (`tag_timeout`) while servoing | level | `hover_thrust` |

`HOVER` ↔ `TAG_IN_SIGHT` flip automatically with detection; the label is the
operator's cue that engagement will happen immediately.

The vertical move only happens while centred within `descend_xy_gate_px`,
which is kept **larger** than `align_tolerance_px` so `ALIGNED` always implies
the climb/descent is allowed.

In `land` mode with `disarm_on_land: true`, the controller disarms once it is
centred and below `land_disarm_height` (odometry altitude), held for
`land_disarm_dwell` seconds.

### Services (all `std_srvs/Trigger`)

| Service | Effect |
|---|---|
| `ibvs/takeoff` | full takeoff: switches to GUIDED_NOGPS, arms, climbs, then holds in `HOVER`. **The only thing that changes the flight mode.** |
| `ibvs/start` | enable servoing |
| `ibvs/stop` | disable servoing, hold in place |

Safety properties baked in:

- Disarming or leaving GUIDED_NOGPS from **any** state drops back to
  `WAIT_ARM` immediately (level attitude, neutral thrust).
- The vehicle never climbs blindly: `CLIMB` is time-boxed
  (`climb_settle_time`), and afterwards vertical motion only happens under a
  fresh detection. No target → `TAG_LOST` → altitude hold.
- `ALIGNED` has hysteresis (`align_hysteresis`) so it does not chatter at the
  tolerance boundary, and requires a dwell time so one lucky sample cannot
  declare success.
- `max_tilt` bounds the commanded lean however big the pixel error gets — it
  is the main safety knob, and it also bounds how much damage a wrong sign
  can do.

Current state is published latched on `ibvs/state` and logged on every
transition.

---

## 5. Control laws

### X-Y: cascade on the image error, with an IMU attitude inner loop

```
error        = image_sign * (detection_px - target_px) / half_dimension   # ±1.0 at the frame edge
desired_tilt = PID_xy(error)                        clamped to ±max_tilt
```

In `attitude` mode the desired tilt is sent as a quaternion and ArduPilot
closes the loop. In `rate` mode an inner loop closes it here:

```
body_rate = kp_att * (desired_tilt - current_tilt)   clamped to ±max_body_rate
```

Attitude comes from `mavros/imu/data` (the FCU's AHRS), which needs no
position solution — it works with no GPS and no OptiTrack. This is the
overshoot fix: the inner loop actively pulls the built-up tilt back toward
level as the error shrinks, which a bare rate command could never do.

**Units:** `kp` is rad of **tilt** per unit frame-fraction, so `kp` *is* the
tilt commanded at the frame edge (`kp 0.14` → 8.0° there, then clamped by
`max_tilt`).

**Why the integrator matters.** A pure-P outer loop must hold a standing error
to balance a constant disturbance (wind, AHRS trim):

```
err_ss = (a_dist / g) / kp        [frame-fractions]
```

| `kp` | 0.15 m/s² | 0.30 m/s² | 0.50 m/s² |
|---|---|---|---|
| 0.06 | 0.25 | 0.51 | 0.85 |
| 0.12 | 0.13 | 0.25 | 0.42 |
| 0.20 | 0.08 | 0.15 | 0.25 |

At `kp 0.06`, 0.3 m/s² of drift needs **half the frame** of standing error —
which is what `2026-08-17-18-30-59.bag` recorded. Hence both fixes: a higher
`kp`, and a non-zero `ki` so a steady disturbance is trimmed out instead of
being balanced by standing error. `i_max` bounds that trim authority (0.05 rad
of tilt cancels ~0.49 m/s² of steady disturbance).

`kd` is the braking term — it commands tilt **away** before arrival. With
`kd = 0` expect overshoot on the way in. `d_max_dt` is the longest detection
gap still differentiated; beyond it the D term is zeroed rather than computed
across the gap.

### Z: open-rate, no range sensor

There is no range sensor, so the vertical axis is open-loop rate through the
`thrust` field: a constant `perch_climb_thrust` while centred on the branch,
or a constant `land_descend_thrust` while centred for landing, clamped to
`[thrust_min, thrust_max]`.

### `max_tilt` — the speed limit

| tilt | max lateral accel | steady approach speed (`g·sin θ/c`, c≈0.75) |
|---|---|---|
| 1° | 0.17 m/s² | 0.23 m/s |
| 2° | 0.34 m/s² | 0.46 m/s |
| 5° | 0.86 m/s² | 1.14 m/s |
| 8.6° | 1.47 m/s² | 1.96 m/s |

---

## 6. Nodes, topics & parameters

### `udp_target_receiver.py`

| Param | Default | Meaning |
|---|---|---|
| `~bind_ip` | `0.0.0.0` | interface to bind |
| `~bind_port` | `5005` | UDP port the PiOS detector sends to |
| `~timeout` | `0.5` | [s] socket timeout (shutdown responsiveness) |
| `~frame_id` | `camera` | `header.frame_id` of the published point |

**Publishes:** `ibvs/target_point` (`geometry_msgs/PointStamped`)

### `ibvs_controller.py`

**Subscribes:** `ibvs/target_point`, `mavros/state`,
`mavros/local_position/odom`, `mavros/imu/data`

**Publishes:** `mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`),
`ibvs/state` (latched `String`), `ibvs/error` (`PointStamped`),
`ibvs/control_angles` (`PointStamped` — x=roll, y=pitch, z=yaw [rad]),
`ibvs/pid_roll` and `ibvs/pid_pitch` (the P/I/D split, for plotting)

Key parameters (all in `ibvs_params_rw.yaml` unless noted):

| Param | Meaning |
|---|---|
| `mission_mode` | `land` or `perch` (**set from the launch**) |
| `image_width` / `image_height` | detector resolution (**set from the launch**) |
| `image_x_sign` / `image_y_sign` | image→body polarity per axis (§3) |
| `engage_on_target` | skip `CLIMB` and servo as soon as armed + GUIDED_NOGPS |
| `control_rate` | [Hz] `AttitudeTarget` publish rate |
| `command_mode` | `attitude` (quaternion) or `rate` (body rate) |
| `max_tilt` | [rad] desired-tilt clamp — **the main safety knob** |
| `pid_xy/{kp,ki,kd,i_max,d_filter,d_max_dt}` | outer-loop gains (§5) |
| `kp_att` / `max_body_rate` | `rate` mode inner loop only |
| `hover_thrust` / `thrust_min` / `thrust_max` | vertical neutral + clamps |
| `perch_climb_thrust` / `land_descend_thrust` | terminal vertical command |
| `disarm_on_land` / `land_disarm_height` / `land_disarm_dwell` | `land` terminal |
| `target_x` / `target_y` | desired offset, frame-fraction (0 = centred) |
| `align_tolerance_px` / `align_dwell_time` / `align_hysteresis` | `ALIGNED` entry/exit |
| `descend_xy_gate_px` | vertical motion only while centred within this |
| `tag_timeout` | [s] no detection within this → `TAG_LOST` |
| `climb_settle_time` / `climb_thrust` / `takeoff_height` | `ibvs/takeoff` path only |

Live plotting: [`config/plotjuggler_ibvs.xml`](config/plotjuggler_ibvs.xml).

---

## 7. Building

```bash
cd ~/uav_ws
catkin build ibvs_perching     # or: catkin_make
source devel/setup.bash
```

Dependencies: `rospy`, `mavros`, `mavros_extras`, and the usual ROS message
packages. `tmuxinator` is needed to run the startup session.

---

## 8. Checking the signs without flying

`startup/real_world/sim_tilt.py` reproduces **exactly** what
`ibvs_controller.py` computes for a given detection pixel, reading the real
`ibvs_params_rw.yaml`. Use it to confirm `image_x_sign` / `image_y_sign` and
the commanded tilt on the bench:

```bash
cd startup/real_world
./sim_tilt.py --sweep     # polarity table: is the sign right?
./sim_tilt.py 900 200     # one-shot, a single detection
./sim_tilt.py             # interactive: type "px py" per line
./sim_tilt.py --rate      # also show rate-mode body rates
```

Run `--sweep` after **any** change to the sign parameters or the camera
mounting.

---

## 9. Troubleshooting

| Symptom | Cause |
|---|---|
| **It climbs away and does not stop** | `GUID_OPTIONS != 0` — thrust is being read as raw thrust. Check `setParam GUID_OPTIONS 0` succeeded in the `stack` window. |
| **`mavros/param/set` rejects with an empty error** | mavros has not finished pulling the FCU parameter table. `waitForParams` in `shell_helpers.sh` blocks for this; it retries. |
| **It leans the wrong way** | Wrong `image_x_sign` / `image_y_sign` for the camera orientation. Confirm with `./sim_tilt.py --sweep` (§8). |
| **The error never settles; it sits at a large standing value** | Pure-P against a steady disturbance (§5). Raise `kp` and enable `ki`. |
| **Nothing happens when the pilot selects GUIDED_NOGPS** | No fresh detection (`ibvs/state` = `HOVER`, not `TAG_IN_SIGHT`), or servoing is disabled — call `ibvs/start`. |
| **`ibvs/target_point` is silent** | PiOS is not sending, or the port/IP is wrong. Check `BIND_PORT` matches the detector, and that the two machines are on the same network. |
| **The error is scaled wrong / saturates early** | `IMAGE_WIDTH`/`IMAGE_HEIGHT` do not match the resolution the detector reports in. Centre the target and read the incoming `px`/`py`. |

Monitoring panes are already open in the `ibvs` tmux window: `ibvs/state`,
`ibvs/target_point`, `ibvs/error`, `ibvs/control_angles`,
`mavros/setpoint_raw/attitude`, `mavros/state`. Every run records a full
`rosbag` in `startup/real_world/`.
