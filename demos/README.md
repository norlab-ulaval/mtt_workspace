# demos

This directory holds the runtime entry points around the workspace.

Use the demo that matches the machine you are on:
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
xhost +local:docker
docker compose run --rm compile
docker compose run --rm bash
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
```

Before using a live demo, run `./scripts/status` from the repo root. The parent repo is only one layer of the workspace state.
