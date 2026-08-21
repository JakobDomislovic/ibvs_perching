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

# Block until mavros has finished downloading the FCU parameter table.
# Setting a parameter before the pull completes makes mavros/param/set reject
# with an empty error ("responded with an error: b''"), which is why
# GUID_OPTIONS used to fail intermittently right after startup.
waitForParams() {
  until timeout 25s rosservice call /$UAV_NAMESPACE/mavros/param/pull "force_pull: false" 2>/dev/null | grep -q "success: True"; do
    echo "waiting for params"
    sleep 2
  done
}

# setParam NAME VALUE -- set an FCU parameter, retrying until it sticks.
setParam() {
  local name=$1 val=$2 i
  for i in $(seq 1 10); do
    if rosrun mavros mavparam -n $UAV_NAMESPACE/mavros set "$name" "$val" 2>/dev/null; then
      echo "setParam: $name=$val OK"
      return 0
    fi
    echo "setParam: retry $name ($i/10)"
    sleep 2
  done
  echo "setParam: ERROR could not set $name after retries"
  return 1
}
