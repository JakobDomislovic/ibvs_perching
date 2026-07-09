# Shell helpers used by session.yml, copied from uav_ros_stack's
# miscellaneous/shell_additions/shell_scripts.sh (credit to
# https://github.com/ctu-mrs/mrs_uav_system) so the real-world startup
# works without the uav_ros_stack installed.

waitForRos() {
  until rostopic list > /dev/null 2>&1; do
    echo "waiting for ros"
    sleep 1;
  done
}

waitForMavros() {
  until timeout 3s rostopic echo /$UAV_NAMESPACE/mavros/state -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for mavros"
    sleep 1;
  done
}

waitForSysStatus() {
  until timeout 3s rostopic echo /$UAV_NAMESPACE/mavros/state -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for /$UAV_NAMESPACE/mavros/state"
    sleep 1;
  done

  while true
    do
      system_status=$(echo "$(rostopic echo /$UAV_NAMESPACE/mavros/state -n 1| grep system_status)" | awk '{print $2}')
      if [[ $system_status == "3" ]]; then
          break
        else
          echo "waiting for system_status"
        fi
      sleep 1
    done
}
