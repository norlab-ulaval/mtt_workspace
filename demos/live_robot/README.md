# live_robot

This folder is the robot-side runtime entry point.

Use it on the robot once the workspace has been synced, the image has been built, and the workspace has been compiled in the container.

If you changed `norlab_robot` launch files or package metadata, rebuild before using this stack:

```bash
docker compose build
docker compose -f ../../compose.yaml run --rm compile
```

`docker compose up` is the default path here. It starts:
- `robot`

The `robot` service brings up:
- the robot description,
- the current `mtt_driver` launch from `src/mtt_core`,
- the sensor stack from `norlab_robot`,
- and mapping after a short delay.

The default assumes the robot already has its host-side Zenoh router and Foxglove bridge running. That is the current state on the real robot, and it avoids binding conflicts on ports `7447` and `8765`.

If those host services are disabled and you want Compose to own them too:

```bash
docker compose --profile infra up
```

That starts:
- `zenoh`
- `robot`
- `foxglove`

This replaces the old habit of manually chaining `weekly.sh`, `com.sh`, and startup scripts when you want the regular live stack.

## Basic use

Start the full robot stack:

```bash
docker compose up
```

Start the robot-side joystick teleop only when you need it:

```bash
docker compose up teleop_robot
```

Open a shell in the runtime image for debugging:

```bash
docker compose --profile debug run --rm bash
```

## Notes

- `teleop_robot` is optional on purpose. The base `robot` service keeps the CAN path and the runtime stack up without assuming a local joystick.
- `docker compose up` does not open Foxglove Studio for you. The bridge is just a websocket server.
- Laptop-side monitoring, laptop-side teleop, and laptop-side bagging now live in `../monitor/`.
- The old scripts under `src/external/norlab_robot/scripts/user_scripts/` are still useful as references, but Compose is now the intended operator-facing entry point.
- If Compose warns about orphan containers after a service layout change, clean them once with:

```bash
docker compose down --remove-orphans
```
