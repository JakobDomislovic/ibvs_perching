# Docker simulation environment

Everything needed to run the `ibvs_perching` simulation in one container:
Gazebo, ArduPilot SITL, mavros and the LARICS `uav_ros_stack` /
[`uav_ros_simulation`](https://github.com/larics/uav_ros_simulation) (pinned
to a known-good commit in the [Dockerfile](Dockerfile)), plus this package,
already built.

## Prerequisites

- [Docker](https://docs.docker.com/engine/install/ubuntu/) (Linux host)
- To show GUIs (Gazebo, rviz, PlotJuggler) on the host, allow X11 access once
  per login session:

  ```bash
  xhost +local:docker
  ```

- Optional, for NVIDIA GPU acceleration:
  [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Quickstart

```bash
git clone https://github.com/JakobDomislovic/ibvs_perching.git
cd ibvs_perching
./docker/build.sh        # first build takes a while (~20-30 min)
./docker/run.sh          # drops you into startup/sim_ibvs inside the container
./start.sh               # tmuxinator session: Gazebo + SITL + mavros + IBVS
```

Detach from the container with `Ctrl-p Ctrl-q` (or just close the terminal);
`./docker/run.sh` re-attaches to the existing container next time.

No GitHub account, SSH key or token is needed: the build clones only public
repositories over HTTPS, and this package itself is copied from your local
checkout.

## Variants and flags

| Command | Effect |
|---|---|
| `./docker/build.sh` | GPU-capable image (`nvidia/opengl` base) → `ibvs_perching:focal` |
| `./docker/build.sh --nogpu` | plain Ubuntu 20.04 base → `ibvs_perching:focal-nogpu` |
| `./docker/run.sh --enable-gpu` | adds `--gpus all` (needs nvidia-container-toolkit) |
| `./docker/run.sh --nogpu` | runs the `focal-nogpu` image |
| `./docker/run.sh --no-mount` | use the code baked into the image instead of the host checkout |
| `./docker/run.sh --run-args "..."` | extra arguments forwarded to `docker run` |

## Developing inside the container

By default `run.sh` mounts your checkout over the package in the container's
workspace (`/root/sim_ws/src/ibvs_perching`), so edits to Python nodes, launch
and config files on the host apply immediately — no image rebuild. After
changing `CMakeLists.txt` or `package.xml`, rebuild inside the container:

```bash
catkin build ibvs_perching
```

Mounts and flags are applied only when the container is first created. To
recreate it (e.g. to switch mount options): `docker rm ibvs_perching_focal`.

## Updating the pinned simulation

The LARICS simulation stack is pinned via the `UAV_ROS_SIM_COMMIT` build arg
in the [Dockerfile](Dockerfile). To move to a newer upstream commit, test it,
then update the default value (or build once with
`./docker/build.sh --build-args "--build-arg UAV_ROS_SIM_COMMIT=<sha>"`).
