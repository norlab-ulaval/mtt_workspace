# mtt_tools

ROS 2 Jazzy workspace for the MTT-154 stack.

This repository is a native `colcon` workspace. You can work with it:
- directly on the host,
- in a regular Docker/Compose workflow,
- or through the VS Code devcontainer.

## Workspace layout

The repository root is the workspace root:

```text
mtt_tools/
  mtt_bringup/
  mtt_description/
  mtt_driver/
  mtt_interfaces/
  mtt_msgs/
  docker/
  compose.yaml
```

Generated artifacts (`build/`, `install/`, `log/`) stay local and are ignored.

## Local development

Tested on Ubuntu 24.04 with ROS 2 Jazzy.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Useful checks:

```bash
ros2 launch mtt_description mtt_description.launch.py
ros2 launch mtt_driver mtt_composable_system.launch.py --show-args
```

## Docker workflow

The repo now ships a regular Docker workflow instead of relying only on VS Code.
It now follows the same spirit as the TIRREX workspace:
- a generated `.env`,
- a reusable multi-stage Dockerfile,
- shared Docker service definitions,
- and a root `compose.yaml` for the interactive dev shell.

Initialize the local Docker environment once:

```bash
./scripts/create_env
```

Run the Docker commands from the repository root so Compose picks up the local `.env`.

Build the development image:

```bash
docker compose -f docker/build.yaml build devel
```

Start an interactive development shell:

```bash
xhost +local:docker
docker compose run --rm bash
```

Inside the container:

```bash
colcon build --symlink-install
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

For local monitoring, reuse the same Foxglove bridge pattern as the robot stack:

```bash
docker compose up monitor
```

Then connect Foxglove Studio to `ws://localhost:8765`.

## VS Code devcontainer

The devcontainer now reuses the same `docker/Dockerfile` as the regular Docker workflow.

1. Open the repo in VS Code.
2. Make sure `Dev Containers` is installed.
3. Run `Dev Containers: Rebuild and Reopen in Container`.

## Notes

- `mtt_description` is the visualization/description package.
- `mtt_bringup` contains simulation and higher-level launch flows.
- `mtt_driver` contains the current Python CAN/teleop/odometry stack.
- For real CAN use, bring interfaces up explicitly on the host before launching the driver.
