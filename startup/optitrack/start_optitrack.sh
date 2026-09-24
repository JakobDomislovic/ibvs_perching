#!/bin/bash
# ============================================================================
# IBVS real-world launcher, PLUS OptiTrack -- EDIT THE CONFIG BLOCK BELOW,
# nothing else.
#
# Identical stack to startup/real_world/start_udp.sh (UDP receiver + IBVS
# controller), plus OptiTrack (uav_ros_general's optitrack.launch). This
# startup launches the controller with use_optitrack:=true, so it
# subscribes DIRECTLY to vrpn_client/estimated_odometry (no relay node in
# between) for its altitude (takeoff_height, hover_height,
# land_disarm_height), instead of mavros/local_position/odom, which never
# gets a real fix with no GPS / no mocap fed into the FCU (see
# ~use_optitrack in ibvs_controller.py). That real altitude is what makes
# mission_mode 'hover' usable here (bench-test IBVS X-Y with Z locked at
# hover_height) -- it is NOT usable under startup/real_world. The same
# vrpn_client topic can also just be watched directly (RViz / PlotJuggler /
# rostopic echo), see the odometry window.
# X-Y is unaffected either way -- that is still pure image-error IBVS.
#
# Needs uav_ros_general + ros_vrpn_client on the ROS_PACKAGE_PATH (the only
# place in this repo that requires them -- startup/real_world needs neither).
#
#   ./start_optitrack.sh
#
# Everything you normally change lives in the CONFIG block right here.
# (Control gains and the RC gate live in custom_config/ibvs_params_rw.yaml.)
# ============================================================================

# ---- CONFIG (the only things you normally change) --------------------------
FCU_URL=/dev/ttyUSB0:921600      # mavros serial link to the flight controller
                                 # (stable alt: /dev/serial/by-id/usb-FTDI_TTL-234X-5V_FT7YLB0N-if00-port0)
MISSION_MODE=hover               # land = down cam, descend+disarm | perch = up cam, climb
                                 # THIS wins over mission_mode in ibvs_params_rw.yaml (the
                                 # launch sets it as an explicit <param> after the rosparam
                                 # load). Switching to perch also needs image_x_sign: -1.0
                                 # in that yaml -- image_y_sign is +1.0 in both modes.
UAV_NAMESPACE=red                # ROS namespace for mavros + ibvs

BIND_PORT=5005                   # UDP port the PiOS detector sends to
# PiOS camera RESOLUTION -- DSJ-3079-HE (USB UVC) @ 1280x720.
# The ONLY camera knowledge the stack needs: the controller normalizes the
# detection by each axis' half-dimension, so the error is +-1.0 at the frame
# edge. No focal length, no calibration. MUST match the resolution the PiOS
# detector reports px/py in (if it downscales before detecting, use the
# DOWNSCALED size). Verify by centering the tag and reading the incoming px/py.
IMAGE_WIDTH=1280                 # [px] frame width  the detector reports in
IMAGE_HEIGHT=720                 # [px] frame height the detector reports in

# OptiTrack -- the controller's altitude source in this startup (see the
# header comment above).
OPTITRACK_IP=192.168.0.50        # OptiTrack/Motive VRPN server IP
OBJECT_NAME=$UAV_NAMESPACE       # rigid-body name registered in Motive
# ---------------------------------------------------------------------------

# work from this script's directory
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

# link the consolidated session file to .tmuxinator.yml
rm -f .tmuxinator.yml
ln session_optitrack.yml .tmuxinator.yml

# hand every knob to tmuxinator as a named setting (read via @settings in the
# session yml) -- so this file is the single source of truth
tmuxinator ibvs_perching_optitrack \
  fcu_url="$FCU_URL" \
  mission_mode="$MISSION_MODE" \
  namespace="$UAV_NAMESPACE" \
  bind_port="$BIND_PORT" \
  image_width="$IMAGE_WIDTH" image_height="$IMAGE_HEIGHT" \
  optitrack_ip="$OPTITRACK_IP" object_name="$OBJECT_NAME"
