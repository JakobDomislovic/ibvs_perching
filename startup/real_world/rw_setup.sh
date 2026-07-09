# Per-aircraft setup, sourced by every tmux pane (see session.yml
# pre_window). Copy this file and pass the copy to start.sh for a
# different aircraft: ./start.sh my_other_uav_setup.sh

# serial link to the flight controller (mavros fcu_url)
export FCU_URL=/dev/ttyUSB_px4:921600

# waitForRos / waitForMavros / waitForSysStatus, package-local so the
# startup does not depend on the uav_ros_stack shell additions
source ./shell_helpers.sh
