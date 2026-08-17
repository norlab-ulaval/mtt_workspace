# Live robot

This is the main robot-side runtime. It can initialize CAN, sensors, joystick
control, mapping and autonomous components. Do not use it as a Docker test.

## Before starting

1. Mechanically secure the robot for the first run.
2. Keep the physical emergency stop reachable.
3. Confirm nobody else is running a robot stack.
4. Confirm the correct CAN, joystick, sensor and network interfaces.
5. Build and run the doctor from the workspace root.

```bash
./scripts/compile
./scripts/status --doctor --summary
```

Inspect effective services without starting them:

```bash
cd demos/live_robot
docker compose config --services
docker compose config --profiles
```

## Default start

```bash
docker compose up
```

The current default stack contains:

- `zenoh`: robot Zenoh router on TCP 7447;
- `robot_driver`: MTT driver, control and CAN path;
- `description`: URDF and static robot description;
- `sensors`: Hesai, RS-Airy, MTi-100 and enabled peripherals;
- `isaac_vslam`: visual odometry sidecar when its image and inputs are ready;
- `mapping`: ICP mapping/localization input;
- `localization`: MTT localization layer;
- `perception`: obstacle/perception components;
- `wiln`: teach-and-repeat runtime, idle until commanded.

Stop with `Ctrl-C`, then verify every container stopped:

```bash
docker compose ps
```

## Configuration

Edit these files first:

```text
demos/live_robot/config/runtime.env
demos/common/config/mtt_driver.yaml
demos/common/config/mtt_control.yaml
demos/common/config/mtt_front_obstacle_monitor.yaml
demos/common/config/mtt_path_follower.yaml
demos/common/config/mtt_repeat_supervisor.yaml
demos/common/config/mtt_route_manager.yaml
demos/common/config/wiln.yaml
```

Important defaults in `runtime.env`:

- sensors, mapping, perception, GPS, joystick and teleop are enabled;
- OAK and MTi-10 are disabled;
- field/Doodle mode is currently enabled;
- COM motor is not part of the default stack, but its profile is configured for
  real EtherCAT unless `COM_MOTOR_DRY_RUN=true` is set.

Treat every `runtime.env` edit as a behavior change.

## Monitoring

Robot-side Foxglove:

```bash
docker compose --profile infra up
```

Connect Foxglove Studio to `ws://<robot-ip>:8765`.

Laptop-side monitoring is normally better over Zenoh:

```bash
cd ../monitor
docker compose up monitor
```

Connect Foxglove Studio to `ws://localhost:8766`.

Read [the Zenoh reference](../../docs/reference/zenoh.md) before Doodle work.

## Checks

Run the checks individually so a failed check is visible:

```bash
docker compose --profile check run --rm field_ready
docker compose --profile check run --rm icp_check
docker compose --profile check run --rm audit_tf
docker compose --profile check run --rm audit_topics
```

These join the live ROS graph but should not publish drive commands.

## Teach and repeat

WILN starts idle in the default stack. The command helpers are opt-in.

Record a new route:

```bash
docker compose run --rm wiln_teach_start
# drive manually
docker compose run --rm wiln_teach_stop
docker compose run --rm wiln_save
```

Validate before replay:

```bash
docker compose run --rm wiln_validate
docker compose run --rm route_preview
docker compose run --rm route_check
```

Replay only after the route, ICP health, command arbitration and physical area
have been checked:

```bash
docker compose run --rm wiln_load
docker compose run --rm wiln_replay
```

Stop replay:

```bash
docker compose run --rm wiln_replay_stop
```

Manual joystick motion with the deadman held is intended to force MANUAL and
cancel replay. The physical emergency stop remains the final safety authority.

## Optional hardware profiles

List the exact services first:

```bash
docker compose --profile '*' config --services
```

High-risk profiles include:

- `manual`: operator-side/manual compatibility tools;
- `wiln-ctrl`: autonomous command helpers;
- `com`: EtherCAT COM motor;
- `field`: field network and diagnostics;
- `perception`: additional perception paths.

For a COM software-only test:

```bash
COM_MOTOR_DRY_RUN=true docker compose --profile com up com_motor
```

Confirm the effective environment in the container logs. Do not assume the
shell override won if `runtime.env` or Compose sets another value.

## Command chain

```text
joy -> cmd_vel/manual_raw -> cmd_vel/manual
WILN/path follower -> controller/cmd_vel
mode manager + mtt_cmd_arbiter_node -> cmd_vel
mtt_can_node -> CAN -> vehicle
```

Only `mtt_cmd_arbiter_node` should publish final `cmd_vel`.

Expected fail-safe behavior:

- e-stop forces neutral and safety lock;
- released deadman removes active manual intent;
- stale final command is neutralized by the driver timeout;
- stale localization should stop autonomous replay;
- startup should remain neutral until valid intent and safety state exist.

These mechanisms require robot-side validation after firmware, CAN or control
changes.

## Shutdown and recovery

Normal shutdown:

```bash
docker compose down
```

After service renames:

```bash
docker compose down --remove-orphans
```

If a container crashes, check that no older project still owns CAN or command
topics before restarting:

```bash
docker ps --format '{{.Names}} {{.Status}}'
```

Do not rely on container restart alone to make the vehicle safe. Use the
physical emergency stop and verify the CAN/controller state.
