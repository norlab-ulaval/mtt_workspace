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

If you plan to work with the real robot, the same script also writes the variables used by the live demos:
- `ROBOT_SSH_TARGET`
- `ROBOT_WORKSPACE`
- `ZENOH_ROUTER_ENDPOINT`
- `FOXGLOVE_WS_URL`

The file is local and ignored by git. It is safe for each person to keep their own robot target.

Examples:

```bash
./scripts/create_env
./scripts/create_env --robot-target mohamed@192.168.2.2
./scripts/create_env --robot-target robot@192.168.2.2
```

If you run `./scripts/create_env` again later, it keeps the current robot target unless you change it or use `--reset-robot-target`.

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

Docker support in this repo is built around:
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
./scripts/autosync_ws
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
docker compose --env-file .env -f demos/live_robot/compose.yaml up monitor
```

For local monitoring, the `monitor` service starts a Foxglove bridge with the same parameters used in the MTT stack:

```bash
docker compose up monitor
```

Then connect Foxglove Studio to `ws://localhost:8765`.

## Robot sync

Use this helper to sync the workspace to the robot:

```bash
./scripts/autosync_ws
```

By default it sends `src`, `docker`, `scripts`, `demos`, `doc`, `compose.yaml`, and `README.md` to `ROBOT_WORKSPACE`. It skips build artifacts, `.vscode`, `.env`, local data, and git internals.

You can also override the target once from the command line:

```bash
./scripts/autosync_ws mohamed@192.168.2.2
./scripts/autosync_ws robot@192.168.2.2
```

When you pass `user@host` without a path, the script now uses `/home/<user>/Project/mtt_ws`.

For a continuous sync loop while you work:

```bash
./scripts/autosync_ws --watch
```

## Live robot demos

The `demos/live_robot/` directory is the laptop-side setup for the real MTT.
It assumes the robot is already up and its Zenoh router and Foxglove bridge are reachable.

A common flow is:

```bash
./scripts/create_env
./scripts/autosync_ws
docker compose -f docker/build.yaml build devel
docker compose --env-file .env -f demos/live_robot/compose.yaml up monitor
```

Then connect Foxglove Studio to `ws://localhost:8766`.

Other useful services:

```bash
docker compose --env-file .env -f demos/live_robot/compose.yaml up teleop_pc
docker compose --env-file .env -f demos/live_robot/compose.yaml up constant_speed
docker compose --env-file .env -f demos/live_robot/compose.yaml up record
```

The `record` service stores a bag under `data/records/live_robot/<timestamp>/` and also saves the demo config, the local git state, the local `.env`, and a small SSH snapshot of the robot runtime. The helper scripts live in `demos/live_robot/scripts/`.

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
