# demos

This directory holds the runtime entry points around the workspace.

Use the demo that matches the machine you are on:
- `common/`
  shared runtime YAML config used by the live demos
- `live_robot/`
  robot-side runtime stack
- `monitor/`
  laptop-side operator stack for a live session
- `data_collection/`
  robot-side bagging workflow with session metadata
- `bag_replay/`
  local replay workflow for recorded sessions
- `description/`
  local description and RViz checks
- `simulation/`
  local simulation stack

## Common commands

From the repository root:

```bash
xhost +si:localuser:$USER
docker compose --profile build -f compose.yaml build devel_image
docker compose run --rm compile
docker compose run --rm bash
```

`compile` only builds the ROS workspace. It does not rebuild images. If the
image is missing, build it explicitly with the `devel_image` profile first.

If you only want the heavy base layer:

```bash
docker compose -f compose.yaml build base
```

On the robot:

```bash
docker compose --env-file .env -f demos/live_robot/compose.yaml up
```

That starts the `robot` service only by default. If the host-side Zenoh router and Foxglove bridge are disabled on the robot, use:

```bash
docker compose --env-file .env -f demos/live_robot/compose.yaml --profile infra up
```

On the operator laptop:

```bash
docker compose --env-file .env -f demos/monitor/compose.yaml up
```

That starts the Foxglove bridge only. Open Foxglove Studio separately and connect to `ws://localhost:8766`.

For local description or simulation checks:

```bash
docker compose --env-file .env -f demos/description/compose.yaml up description
docker compose --env-file .env -f demos/simulation/compose.yaml up simulation
docker compose --env-file .env -f demos/simulation/compose.yaml up rviz
docker compose --env-file .env -f demos/simulation/compose.yaml up foxglove
docker compose --env-file .env -f demos/simulation/compose.yaml up control
```

Before using a live demo, run `./scripts/status` from the repo root. The parent repo is only one layer of the workspace state.
