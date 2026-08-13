#!/bin/bash
# ============================================================================
# IBVS DIRECTION VISUALIZER -- OpenCV window (black screen + arrow) showing
# which way the controller commands the UAV to move, from live PiOS input.
# Needs a display (Pi desktop / X forwarding). Press q or ESC to quit.
#
# TWO MODES (set WITH_REAL_FCU below):
#
#  0 = STANDALONE bench (no FCU at all): starts roscore + receiver + controller
#      + a full FCU stub (fake armed+GUIDED + level odom). Pure logic test.
#
#  1 = REAL FCU: you already run the real stack (mavros + receiver + controller,
#      e.g. via start_udp.sh) and ARM with the RC controller. This script then
#      only adds a level-odom source (indoors there is no GPS, so the EKF gives
#      no local_position/odom and the controller's attitude cascade -- hence the
#      body rates -- would stay zero) plus the visualizer window.
#
# !!! bench_fcu_stub fakes a level odom. PROPS OFF only. !!!
# ============================================================================

# ---- CONFIG ----------------------------------------------------------------
WITH_REAL_FCU=1            # 1 = real FCU armed by pilot | 0 = standalone bench
MISSION_MODE=land          # land = down cam | perch = up cam (standalone only)
UAV_NAMESPACE=red
BIND_PORT=5005             # UDP port PiOS sends {"px","py"} to (standalone only)
# DSJ-3079-HE @ 1280x720:
CX=640.0; CY=360.0; FX=251.0; FY=251.0
BENCH_ALT=1.5             # fake altitude the odom stub reports
FULL_SCALE_RATE=0.35      # rad/s that draws a full-length arrow (=max_body_rate)
# ---------------------------------------------------------------------------

set +e
source /opt/ros/noetic/setup.bash
source /root/uav_ws/devel/setup.bash 2>/dev/null
export UAV_NAMESPACE
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}

STUB_PID=""; LAUNCH_PID=""; RC_STARTED=""
cleanup() {
  echo; echo "shutting down visualizer..."
  [ -n "$STUB_PID" ] && kill $STUB_PID 2>/dev/null
  [ -n "$LAUNCH_PID" ] && kill $LAUNCH_PID 2>/dev/null
  pkill -9 -f ibvs_bench_viz 2>/dev/null
  pkill -9 -f bench_fcu_stub 2>/dev/null
  if [ "$WITH_REAL_FCU" != "1" ]; then
    pkill -9 -f udp_target_receiver 2>/dev/null
    pkill -9 -f ibvs_controller 2>/dev/null
  fi
  [ -n "$RC_STARTED" ] && { pkill -9 -f roscore 2>/dev/null; pkill -9 -f rosmaster 2>/dev/null; }
}
trap cleanup EXIT INT TERM

master_up() { python3 -c "import xmlrpc.client,socket;socket.setdefaulttimeout(2);xmlrpc.client.ServerProxy('$ROS_MASTER_URI').getPid('/x')" 2>/dev/null; }

if [ "$WITH_REAL_FCU" = "1" ]; then
  # ---- REAL FCU: expect the stack already running; add odom + viz ----------
  if ! master_up; then
    echo "ERROR: no ROS master at $ROS_MASTER_URI."
    echo "Start your stack first (e.g. ./start_udp.sh), then run this."
    exit 1
  fi
  echo "REAL-FCU mode: adding a level-odom source (publish_state:=false)."
  echo "  (arm + set GUIDED_NOGPS with your RC controller; PROPS OFF)"
  rosrun ibvs_perching bench_fcu_stub.py \
    _publish_state:=false _publish_odom:=true _fake_services:=false \
    _altitude:="$BENCH_ALT" __ns:="$UAV_NAMESPACE" >/tmp/bench_odom.log 2>&1 &
  STUB_PID=$!
  sleep 1
else
  # ---- STANDALONE bench: fake the whole FCU -------------------------------
  if ! master_up; then
    echo "starting roscore..."
    roscore >/tmp/bench_roscore.log 2>&1 &
    RC_STARTED=1
    for i in $(seq 1 20); do master_up && break; sleep 1; done
  fi
  echo "STANDALONE mode: receiver + controller + full FCU stub."
  roslaunch ibvs_perching ibvs_bench_viz.launch \
    namespace:="$UAV_NAMESPACE" mission_mode:="$MISSION_MODE" \
    bind_port:="$BIND_PORT" cx:="$CX" cy:="$CY" fx:="$FX" fy:="$FY" \
    bench_altitude:="$BENCH_ALT" >/tmp/bench_launch.log 2>&1 &
  LAUNCH_PID=$!
  sleep 4
fi

echo "opening visualizer window (q / ESC to quit)..."
rosrun ibvs_perching ibvs_viz.py \
  _namespace:="$UAV_NAMESPACE" _mission_mode:="$MISSION_MODE" \
  _full_scale_rate:="$FULL_SCALE_RATE"
