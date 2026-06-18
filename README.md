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
- `.gitmodules`
  Canonical source of truth for nested repositories and exact Git pointers.
- `dependencies/robot.repos`
  Mirror manifest for `vcs` users and legacy bootstrap flows.

## Before you start

Host requirements:
- Ubuntu with Docker installed and working
- access to the private repositories listed in `.gitmodules`
- `vcstool` installed as `vcs` only if you need the legacy `dependencies/robot.repos` flow

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
```

Log out and back in after adding yourself to the `docker` group. Verify access
without `sudo`:

```bash
docker info
```

## Get the workspace

Clone and initialize the complete workspace:

```bash
git clone git@github.com:norlab-ulaval/mtt_workspace.git
cd mtt_workspace
./scripts/create_ws
./scripts/compile
```

`create_ws` creates `.env` with the current host UID/GID and initializes every
nested submodule. `compile` verifies Docker access, rebuilds the local image
when its user does not match the host UID/GID, then builds the ROS workspace.

Equivalent Git-only submodule command:

```bash
git submodule update --init --recursive
```

Check the result:

```bash
./scripts/status
```

## Build

### Docker build

This is the standard path for this workspace.

For normal use, including the first build on a new machine:

```bash
./scripts/compile
```

The script rebuilds `mtt_workspace:base` and `mtt_workspace:devel` when the
development image is absent or was built for another UID/GID. This is required
because the container writes directly into the bind-mounted workspace.

Advanced explicit image builds:

```bash
docker compose --profile build -f compose.yaml build base devel_image
docker compose -f compose.yaml build base
```

Direct ROS build without the preflight checks:

```bash
docker compose -f compose.yaml run --rm compile
```

Open a shell in the runtime image:

```bash
docker compose -f compose.yaml run --rm bash
```

If the image is missing, `compile` and `bash` will fail instead of rebuilding.
Prefer `./scripts/compile`, which handles this automatically.

### Installation troubleshooting

`permission denied` when running Docker:

```bash
sudo usermod -aG docker "$USER"
```

Log out and back in, then check `docker info`.

`permission denied` under `build/`, `install/`, `log/`, or `.ccache/` means
those generated directories belong to a different UID. `./scripts/compile`
detects this and prints the exact `chown` command to repair them.

`not our ref` or an empty nested dependency means a submodule pointer is not
available from its configured remote. Rerun `./scripts/create_ws`; it now
validates every recursive submodule and reports the exact failing path.

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
- The parent repo pins all nested repositories through submodules. Keep `.gitmodules` and `dependencies/robot.repos` aligned when adding or removing dependencies.
- Do not assume the parent repo alone describes the full runtime. `norlab_robot` still owns a large part of the live assembly.
- If something works in one demo and not another, compare the demo-owned config first before changing package code.
