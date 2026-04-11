# live_robot

PC-side demo stack for talking to the real MTT over Zenoh and keeping a local record of the session.

It is meant for the operator laptop:
- joystick on the laptop,
- monitor on the laptop,
- bags saved on the laptop,
- the real CAN path stays on the robot.

## Services

- `bash`: a shell already pointed at the robot Zenoh router
- `monitor`: a local Foxglove bridge that mirrors the robot topics to `ws://localhost:8766`
- `teleop_pc`: joystick on the laptop, commands sent to the robot on `cmd_vel/teleop`
- `constant_speed`: simple repeated command on `/controller/cmd_vel`
- `record`: local rosbag with config, git state, env snapshot, and a small robot-side snapshot

## Before you start

From the repository root:

```bash
./scripts/create_env
./scripts/autosync_ws
docker compose run --rm compile
```

For the default lab login, this is enough. If your robot user is different, set it once:

```bash
./scripts/create_env --robot-target <robot_user>@192.168.2.2
```

The target is kept in your local `.env`, so you do not need to edit tracked files.

## Basic use

Start the local Foxglove bridge:

```bash
docker compose --env-file .env -f demos/live_robot/compose.yaml up monitor
```

Then connect Foxglove Studio to:

```text
ws://localhost:8766
```

Start joystick teleop from the laptop:

```bash
docker compose --env-file .env -f demos/live_robot/compose.yaml up teleop_pc
```

Send a fixed command instead:

```bash
MTT_LINEAR_SPEED=0.15 MTT_ANGULAR_SPEED=0.0 MTT_COMMAND_DURATION=5 \
docker compose --env-file .env -f demos/live_robot/compose.yaml up constant_speed
```

Record a session locally:

```bash
docker compose --env-file .env -f demos/live_robot/compose.yaml up record
```

Preview the bag command without recording:

```bash
docker compose --env-file .env -f demos/live_robot/compose.yaml \
  run --rm bash python3 demos/live_robot/scripts/record.py --config demos/live_robot/config/records.yaml --dry-run
```

The record goes under `data/records/live_robot/<timestamp>/` with:
- the bag itself,
- the demo config,
- git branch / commit / diff,
- environment snapshot,
- and a small robot-side snapshot captured over SSH.

## Notes

- `teleop_pc` only runs the joystick side. It does not open CAN locally.
- If you want joystick on the robot instead, keep using the robot-side launch there.
- `constant_speed` publishes on `/controller/cmd_vel`, which keeps it separate from the teleop path.
- The demo-local helpers live in `demos/live_robot/scripts/`.
