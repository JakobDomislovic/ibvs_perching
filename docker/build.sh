#!/bin/bash
# Build the ibvs_perching simulation image.
#
# Usage:
#   ./docker/build.sh                  # GPU-capable image (nvidia/opengl base) -> ibvs_perching:focal
#   ./docker/build.sh --nogpu          # plain Ubuntu base                      -> ibvs_perching:focal-nogpu
#   ./docker/build.sh --build-args "--build-arg UAV_ROS_SIM_COMMIT=<sha>"

set -e

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
REPO_ROOT=`dirname "$MY_PATH"`

distro="focal"
base_image="nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04"
build_args=""

for (( i=1; i<=$#; i++));
do
  param="${!i}"

  if [ "$param" == "--nogpu" ]; then
    distro="focal-nogpu"
    base_image="ubuntu:20.04"
  fi

  if [ "$param" == "--build-args" ]; then
    j=$((i+1))
    build_args="${!j}"
  fi

done

echo "Building ibvs_perching:$distro (base: $base_image)"

docker build \
  -f "$MY_PATH/Dockerfile" \
  --build-arg BASE_IMAGE=$base_image \
  $build_args \
  -t ibvs_perching:$distro \
  "$REPO_ROOT"
