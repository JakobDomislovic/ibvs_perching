#!/bin/bash
# Run (or re-attach to) the ibvs_perching simulation container.
# Adapted from larics/uav_ros_simulation run_docker.sh.
#
# Usage:
#   ./docker/run.sh                    # run ibvs_perching:focal, repo mounted for live editing
#   ./docker/run.sh --enable-gpu       # pass --gpus all (needs nvidia-container-toolkit)
#   ./docker/run.sh --nogpu            # run the ibvs_perching:focal-nogpu image
#   ./docker/run.sh --no-mount         # use the code baked into the image instead of the host checkout
#   ./docker/run.sh --run-args "..."   # extra args forwarded to docker run
#
# NOTE: mounts and flags only apply the first time the container is created.
# To start over: docker rm ibvs_perching_<distro>

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
REPO_ROOT=`dirname "$MY_PATH"`

echo "Running Docker Container"
CONTAINER_NAME=ibvs_perching

distro="focal"
run_args=""
gpu_enabled=""
mount_repo=true

for (( i=1; i<=$#; i++));
do
  param="${!i}"

  if [ "$param" == "--enable-gpu" ]; then
    gpu_enabled="--gpus all"
  fi

  if [ "$param" == "--nogpu" ]; then
    distro="focal-nogpu"
  fi

  if [ "$param" == "--no-mount" ]; then
    mount_repo=false
  fi

  if [ "$param" == "--run-args" ]; then
    j=$((i+1))
    run_args="${!j}"
  fi

done

# Mount the host checkout over the baked-in copy: rospy scripts, launch and
# config files take effect without rebuilding the image. Run catkin build
# inside the container after changing CMakeLists.txt / package.xml.
mount_args=""
if [ "$mount_repo" = true ]; then
  mount_args="--volume $REPO_ROOT:/root/sim_ws/src/ibvs_perching"
fi

run_args="$gpu_enabled $mount_args $run_args"

echo "Running in $distro"

# Check if there is an already running container with the same distro
full_container_name="${CONTAINER_NAME}_${distro}"
running_container="$(docker container ls --all | grep $full_container_name)"
if [ -z "$running_container" ]; then
  echo "Running $full_container_name for the first time!"
else
  echo "Found an open $full_container_name container. Starting and attaching!"
  eval "docker start $full_container_name"
  eval "docker attach $full_container_name"
  exit 0
fi

# Forward the SSH agent when one is available (useful for pushing to your
# fork from inside the container); not required to build or run anything.
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ssh_args=""
if [ -n "$SSH_AUTH_SOCK" ]; then
  mkdir -p ~/.ssh
  ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
  ssh_args="--volume $HOME/.ssh/ssh_auth_sock:/ssh-agent --env SSH_AUTH_SOCK=/ssh-agent"
fi

set -x
docker run \
  $run_args \
  $ssh_args \
  -it \
  --network host \
  --privileged \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="XAUTHORITY=${XAUTH}" \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $full_container_name \
  ibvs_perching:$distro \
  /bin/bash
