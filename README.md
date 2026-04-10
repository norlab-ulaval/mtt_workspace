# mtt_tools

ROS 2 Jazzy workspace for the MTT-154 platform.

This repository contains the current MTT bringup, description, driver, interfaces, and messages in a regular `colcon` workspace.

You can work with it:
- directly on the host,
- in Docker with the local compose workflow,
- or through the VS Code devcontainer.

## Quick start

Start in the repository root and create the local Docker environment once:

```bash
./scripts/create_env
```

For host-side development:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

If you are coming from the old flat layout, clean generated artifacts once before rebuilding:

```bash
rm -rf build install log
```

For Docker-first development:

```bash
./scripts/create_env
docker compose -f docker/build.yaml build devel
docker compose run --rm bash
```

Inside the container:

```bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

## Workspace layout

The repository root is the workspace root:

```text
mtt_tools/
  src/
    mtt_bringup/
    mtt_description/
    mtt_driver/
    mtt_interfaces/
    mtt_msgs/
  docker/
  demos/
  doc/
  data/
  compose.yaml
```

Generated artifacts (`build/`, `install/`, `log/`) stay local and are ignored.

## Local development

Tested on Ubuntu 24.04 with ROS 2 Jazzy.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

Useful checks:

```bash
ros2 launch mtt_description mtt_description.launch.py
ros2 launch mtt_driver mtt_composable_system.launch.py --show-args
```

## Docker workflow

The repo ships a regular Docker workflow for local MTT work:
- `./scripts/create_env` prepares `.env` and the host-side bind mounts,
- `docker/build.yaml` builds the development and full images,
- `compose.yaml` provides the interactive shell, compile flow, and monitor service.

Start with:

```bash
./scripts/create_env
docker compose -f docker/build.yaml build devel
```

Run the Docker commands from the repository root so Compose picks up the local `.env`.

Start an interactive development shell:

```bash
xhost +local:docker
docker compose run --rm bash
```

Inside the container:

```bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

Build the fully compiled image:

```bash
docker compose -f docker/build.yaml build full
```

Small helpers:

```bash
./scripts/status
./scripts/pull
```

Useful compose services:

```bash
docker compose run --rm bash
docker compose run --rm compile
docker compose run --rm dev
docker compose up monitor
```

Demo wrappers are available under `demos/` and can be launched from the workspace root:

```bash
xhost +local:docker
docker compose --env-file .env -f demos/description/compose.yaml up description
docker compose --env-file .env -f demos/simulation/compose.yaml up simulation
docker compose --env-file .env -f demos/monitor/compose.yaml up monitor_demo
```

For local monitoring, the `monitor` service starts a Foxglove bridge with the same parameters used in the MTT stack:

```bash
docker compose up monitor
```

Then connect Foxglove Studio to `ws://localhost:8765`.

## VS Code devcontainer

The devcontainer now reuses the same `docker/Dockerfile` as the regular Docker workflow.
Its internal `colcon` cache is kept separate from the host `build/`, `install/`, and `log/` directories so reopening the project in VS Code does not fight with a host-side build.

1. Open the repo in VS Code.
2. Make sure `Dev Containers` is installed.
3. Run `Dev Containers: Rebuild and Reopen in Container`.

## Notes

- `mtt_description` is the visualization/description package.
- `mtt_bringup` contains simulation and higher-level launch flows.
- `mtt_driver` contains the current Python CAN/teleop/odometry stack.
- All ROS packages now live under `src/`.
- For real CAN use, bring interfaces up explicitly on the host before launching the driver.
