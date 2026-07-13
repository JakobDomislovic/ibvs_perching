# Simulation image for ibvs_perching.
#
# Modeled on larics/uav_ros_simulation Dockerfile.binary: stage 0 provides the
# prebuilt uav_ros_stack binary workspace, stage 1 builds the simulation
# workspace on top of it and adds this package.
#
# Build from the repo root (build.sh does this):
#   docker build -f docker/Dockerfile -t ibvs_perching:focal .

ARG VERSION=0.2.3
ARG DISTRO=focal
ARG BASE_IMAGE=nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04

FROM lmark1/uav_ros_stack:$DISTRO-bin-$VERSION

FROM $BASE_IMAGE

ARG CATKIN_WORKSPACE=sim_ws
ARG HOME=/root
ARG USER=root
# Known-good larics/uav_ros_simulation commit for this package; bump
# deliberately after testing, so student builds stay reproducible.
ARG UAV_ROS_SIM_COMMIT=e0776f402207f913064b211aafb0594e68a4a5ce
ENV DEBIAN_FRONTEND=noninteractive
ENV GITHUB_RUNNER=true
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    git \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    curl \
    vim \
    tzdata \
    openssh-client \
    bash-completion && \
    dpkg-reconfigure --frontend noninteractive tzdata

# All larics repos involved are public: rewrite the SSH URLs used by
# gitman.yml and the install scripts to HTTPS, so building the image needs
# no SSH keys or agent forwarding.
RUN git config --global url."https://github.com/".insteadOf "git@github.com:"

# Prebuilt uav_ros_stack workspace from stage 0
WORKDIR $HOME
COPY --from=0 $HOME/uav_ws $HOME/uav_ws

# Install uav_ros_simulation (pinned) and set up the catkin workspace
RUN git clone https://github.com/larics/uav_ros_simulation.git && \
    git -C uav_ros_simulation checkout $UAV_ROS_SIM_COMMIT && \
    ./uav_ros_simulation/installation/install_and_setup_workspace.sh --workspace $CATKIN_WORKSPACE --sparse

WORKDIR $HOME/$CATKIN_WORKSPACE/src
SHELL ["/bin/bash", "-c"]

# Add sourcing
RUN echo "source $HOME/uav_ws/install/setup.bash" >> $HOME/.bashrc && \
  # 1) remove sim_ws sourcing, need to add it after..
  sed -i "s|source $HOME/$CATKIN_WORKSPACE/devel/setup.bash||g" $HOME/.bashrc

# Build the simulation workspace
RUN source $HOME/.bashrc && \
  catkin clean --yes && \
  catkin config --extend $HOME/uav_ws/install && \
  catkin build --limit-status-rate 0.2

# 2) .. add source sim_ws NOW
RUN echo "source $HOME/$CATKIN_WORKSPACE/devel/setup.bash" >> $HOME/.bashrc && \
  # 3) Expose uav_ws packages
  echo 'export ROS_PACKAGE_PATH='"$HOME"'/uav_ws/install/share:$ROS_PACKAGE_PATH' >> $HOME/.bashrc

# ibvs_perching comes from the local checkout (build context = repo root),
# so the image always matches the code and needs no GitHub auth. Kept as the
# last layers: editing the package only rebuilds from here.
COPY . $HOME/$CATKIN_WORKSPACE/src/ibvs_perching

RUN source $HOME/.bashrc && \
  catkin build ibvs_perching --limit-status-rate 0.2

# Prepare for startup
WORKDIR $HOME/$CATKIN_WORKSPACE/src/ibvs_perching/startup/sim_ibvs
