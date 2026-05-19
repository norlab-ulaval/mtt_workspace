# MTT workspace

ROS 2 Jazzy workspace for the MTT robot.

![MTT Robot](./mtt.jpg)

This repository is the workspace shell around the robot software. It owns:
- Docker images and Compose entry points
- dependency manifests
- demo configurations
- helper scripts
- workspace-level documentation

The ROS packages themselves are split across nested repositories under `src/`.

## What is in this workspace

```text
mtt_workspace/
  src/
    mtt_core/
    external/
  dependencies/
    robot.repos
  docker/
  demos/
  documentations/
  scripts/
```

- `src/mtt_core`
  MTT-owned ROS packages: driver, control, description, bringup, messages, interfaces.
- `src/external`
  Imported dependencies and runtime overlays.
- `src/external/norlab_robot`
  Robot runtime integration: sensors, mapping, recording, Foxglove, teach-and-repeat.
- `dependencies/robot.repos`
  Source of truth for the imported repositories expected in this workspace.

## Before you start

Host requirements:
- Ubuntu with Docker installed and working
- ROS 2 Jazzy available on the host if you want to build outside Docker
- `vcstool` installed as `vcs`
- access to the private repositories listed in `dependencies/robot.repos`

Typical host setup:

```bash
sudo apt-get update
sudo apt-get install -y python3-vcstool
```

Typical Docker setup:

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-plugin
sudo usermod -aG docker "$USER"
newgrp docker
```

## Get the workspace

Clone the workspace shell:

```bash
git clone git@github.com:norlab-ulaval/mtt_workspace.git
cd mtt_workspace
```

Create the local environment files and bind-mount directories:

```bash
./scripts/create_env
```

Import the nested repositories:

```bash
./scripts/create_ws
```

Check the result:

```bash
./scripts/status
```

## Build

### Docker build

This is the standard path for this workspace.

Build images explicitly when needed (new clone, Dockerfile change, or missing image):

```bash
docker compose --profile build -f compose.yaml build devel_image
```

If you need the heavy base image explicitly:

```bash
docker compose -f compose.yaml build base
```

Build the ROS workspace (no image rebuild):

```bash
docker compose -f compose.yaml run --rm compile
```

Open a shell in the runtime image:

```bash
docker compose -f compose.yaml run --rm bash
```

If the image is missing, `compile` and `bash` will fail instead of rebuilding.
Run the explicit build once, then continue with `compile`.

### Docker images (what they contain)

- `mtt_workspace:base`
  heavy system layer (ZED SDK, depthai, libpointmatcher, OS tools). Big and slow
  to rebuild, so we keep it stable.
- `mtt_workspace:devel`
  adds ROS packages and Python deps needed by the workspace. This is the image
  used by `compile`, `bash`, `dev`, and demo services.

The container always bind-mounts the local workspace, so your local `src/` is the
source of truth at runtime.

### Platform honesty (where it works)

- Works on Linux x86_64 with Docker.
- GPU support is optional and needs the NVIDIA container runtime on the host.
- X11 GUI works on Linux when the host allows it.

Not guaranteed:
- macOS and Windows (host networking and GPU passthrough are Linux-only)
- ARM hosts (images are built for x86_64)

### Host build

Use this only if you really want a host-side build.

One-shot setup for a clean Ubuntu host:

```bash
./scripts/install_host.sh --with-ros
```

If you already have ROS 2 Jazzy installed:

```bash
./scripts/install_host.sh
```

Dry-run or check-only:

```bash
./scripts/install_host.sh --dry-run
./scripts/install_host.sh --check
```

What the script checks (host-only path):
- ROS 2 Jazzy setup
- rosdep, colcon, vcs
- ZED SDK presence under /usr/local/zed
- depthai headers under /usr/local/include/depthai
- libpointmatcher and libnabo in the linker cache

Honest limitation: the script cannot install vendor SDKs (ZED, depthai) for you.
It only detects them. These are provided by the Docker base image; on host you
must install them manually if you want a full native build.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/mtt_core src/external
source install/setup.bash
```

## Runtime model

The live stack is assembled from demo-owned configuration. Runtime tuning should happen in the demo YAML files first, not in package defaults under `src/**/config`.

Current runtime ownership:
- `demos/common/config/`
  shared runtime tuning
- `demos/data_collection/config/`
  data collection demo settings
- `demos/live_robot/config/`
  live robot demo settings
- `demos/bag_replay/`
  offline replay and reconstruction

The control path is intended to stay simple:

```text
manual intent   -> cmd_vel/manual_raw -> cmd_vel/manual
auto intent     -> controller/cmd_vel
arbiter         -> cmd_vel
driver          -> mtt_can_node
```

Only the final arbiter should publish the final `cmd_vel`.

## Main demos

Start from [demos/README.md](./demos/README.md).

Most common entry points:
- `demos/live_robot`
  robot-side live runtime
- `demos/data_collection`
  robot-side recording workflow
- `demos/monitor`
  laptop-side monitoring and operator tools
- `demos/bag_replay`
  offline replay and reconstruction

## Useful workspace commands

```bash
./scripts/status
./scripts/pull
./scripts/autosync_ws
```

- `status`
  shows the parent repo and the nested repos together
- `pull`
  updates the parent repo and nested repos
- `autosync_ws`
  pushes the workspace to a robot target with `rsync`

Because this workspace uses nested repositories, root-level `git status` is not enough to decide whether the workspace is clean.

## Documentation layout

All technical notes live in:
- [documentations/README.md](./documentations/README.md)

## Notes

- Do not treat package default YAML files under `src/` as the normal operator surface.
- Do not assume the parent repo alone describes the full runtime. `norlab_robot` still owns a large part of the live assembly.
- If something works in one demo and not another, compare the demo-owned config first before changing package code.
