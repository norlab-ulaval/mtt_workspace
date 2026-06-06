# monitor

This folder is the laptop-side operator stack for a live robot session.

`docker compose up` starts the local monitor bridge only. It does not open a UI window by itself.

## Basic use

Start the local monitor bridge:

```bash
docker compose up
```

Then connect Foxglove Studio to:

```text
ws://localhost:8766
```

If you prefer the robot-side bridge, connect Foxglove Studio directly to `ROBOT_FOXGLOVE_URL` instead.

Start laptop-side joystick teleop:

```bash
docker compose up teleop_pc
```

Publish only the laptop joystick as `/joy` and let the robot-side control stack
apply the normal MTT mapping/safety logic:

```bash
docker compose up joy_pc
```

For a stable device path, use:

```bash
JOY_DEVICE=/dev/input/by-id/<your-joystick> docker compose up joy_pc
```

Send a fixed command instead:

```bash
docker compose up constant_speed
```

Record a session on the laptop:

```bash
docker compose up record
```

Open a shell already pointed at the robot Zenoh session:

```bash
docker compose --profile debug run --rm bash
```

## Notes

- This stack talks to the robot over Zenoh. It does not bring up CAN or sensors locally.
- The robot-side runtime now lives in `../live_robot/`.
