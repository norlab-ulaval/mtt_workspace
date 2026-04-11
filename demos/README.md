# demos

This directory is reserved for reproducible local scenarios and launch recipes.

Run these from the repository root so Compose picks up the workspace `.env`:

```bash
xhost +local:docker
docker compose --env-file .env -f demos/description/compose.yaml up description
docker compose --env-file .env -f demos/simulation/compose.yaml up simulation
docker compose --env-file .env -f demos/monitor/compose.yaml up monitor_demo
docker compose --env-file .env -f demos/live_robot/compose.yaml up monitor
```

The root services remain available too:

```bash
docker compose run --rm bash
docker compose run --rm compile
docker compose up monitor
```

The `live_robot/` demo is the laptop-side workflow for the real MTT:
- `monitor` runs a local Foxglove bridge and subscribes to the robot through Zenoh,
- `teleop_pc` publishes joystick commands from your laptop into `cmd_vel/teleop`,
- `constant_speed` publishes a fixed `TwistStamped` for short checks,
- `record` saves a bag plus the local config, git state, and a remote runtime snapshot under `data/records/`.
