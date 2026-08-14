#!/bin/bash
# ============================================================================
# IBVS real-world launcher -- EDIT THE CONFIG BLOCK BELOW, nothing else.
#
# Brings up the WHOLE stack + UDP receiver + IBVS controller in one tmux
# session (session_udp.yml). Off-board detection over UDP; no on-board
# ArUco / RealSense / uav_ros_stack / OptiTrack.
#
#   ./start_udp.sh
#
# Everything you normally change lives in the CONFIG block right here.
# (Control gains and the RC gate live in custom_config/ibvs_params_rw.yaml.)
# ============================================================================

# ---- CONFIG (the only things you normally change) --------------------------
FCU_URL=/dev/ttyUSB0:921600      # mavros serial link to the flight controller
                                 # (stable alt: /dev/serial/by-id/usb-FTDI_TTL-234X-5V_FT7YLB0N-if00-port0)
MISSION_MODE=land                # land = down cam, descend+disarm | perch = up cam, climb
UAV_NAMESPACE=red                # ROS namespace for mavros + ibvs

BIND_PORT=5005                   # UDP port the PiOS detector sends to
# PiOS camera geometry -- DSJ-3079-HE (USB UVC) @ 1280x720.
# These MUST match the resolution the PiOS detector reports px/py in (if it
# downscales before detection, use the DOWNSCALED size here, not the capture
# size). cx/cy go to the UDP receiver (absolute px -> center-relative pixel
# error, exact resolution math); fx/fy go to the CONTROLLER, which turns that
# pixel error into body-frame metres. fx/fy are NOT calibrated for the DSJ --
# they scale the X-Y loop gain, so a wrong value reads as mis-tuned lateral
# response. Verify by centering the tag and reading the incoming px/py.
CX=640.0                         # image center x [px] = width/2
CY=360.0                         # image center y [px] = height/2
FX=251.0                         # focal length x [px] -- TODO: uncalibrated, carried
FY=251.0                         # focal length y [px]    over from the Nicla camera
# ---------------------------------------------------------------------------

# work from this script's directory
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

# link the consolidated session file to .tmuxinator.yml
rm -f .tmuxinator.yml
ln session_udp.yml .tmuxinator.yml

# hand every knob to tmuxinator as a named setting (read via @settings in the
# session yml) -- so this file is the single source of truth
tmuxinator ibvs_perching_udp \
  fcu_url="$FCU_URL" \
  mission_mode="$MISSION_MODE" \
  namespace="$UAV_NAMESPACE" \
  bind_port="$BIND_PORT" \
  cx="$CX" cy="$CY" fx="$FX" fy="$FY"
