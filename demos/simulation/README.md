# simulation

Local simulation entry points live here.

Use this demo instead of the repository-root `compose.yaml` when you want Gazebo, RViz, Foxglove, or the operator control stack for simulation.

## Services

- `simulation`
  starts Gazebo and the robot runtime
- `control`
  starts the shared `mtt_control` operator stack for simulation
- `rviz`
  starts RViz against the simulation clock
- `foxglove`
  starts the Foxglove bridge

## Common commands

From the repository root:

```bash
docker compose -f demos/simulation/compose.yaml up -d simulation
docker compose -f demos/simulation/compose.yaml up -d rviz
docker compose -f demos/simulation/compose.yaml up -d foxglove
docker compose -f demos/simulation/compose.yaml up -d control
```

Start the simulator first, then add the other services one by one.

Useful environment overrides:

```bash
SIM_HEADLESS=false
SIM_ENABLE_OPERATOR_CONTROL=false
SIM_CONTROL_ENABLE_JOYSTICK=true
SIM_JOY_DEVICE=/dev/input/js0
SIM_JOY_DEADZONE=0.15
```
