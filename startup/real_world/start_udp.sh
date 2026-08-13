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
# PiOS camera intrinsics -- Arduino Nicla Vision (GC2145) @ QVGA 320x240:
CX=160.0                         # image center x [px]
CY=120.0                         # image center y [px]
FX=251.0                         # focal length x [px]
FY=251.0                         # focal length y [px]
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
