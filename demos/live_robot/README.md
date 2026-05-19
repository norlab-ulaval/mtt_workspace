# live_robot

This folder is the robot-side runtime entry point.

Use it on the robot once the workspace has been synced, the image has been built, and the workspace has been compiled in the container.

If you changed `norlab_robot` launch files or package metadata, rebuild before using this stack:

```bash
docker compose build
docker compose -f ../../compose.yaml run --rm compile
```

`docker compose up` is the default path here. It starts:
- `zenoh`
- `robot`

The `robot` service brings up:
- the robot description,
- the current `mtt_driver` launch from `src/mtt_core`,
- the sensor stack from `norlab_robot`,
- and mapping after a short delay.

Foxglove stays opt-in under `--profile infra`:

```bash
docker compose --profile infra up
```

That starts:
- `robot`
- `foxglove`

If you want RViz from the same stack:

```bash
docker compose --profile viz up rviz
```

This replaces the old habit of manually chaining `weekly.sh`, `com.sh`, and startup scripts when you want the regular live stack.

## Basic use

Start the full robot stack:

```bash
docker compose up
```

Edit the operator-facing runtime settings here:

```bash
demos/live_robot/config/runtime.env
demos/common/config/mtt_driver.yaml
demos/common/config/mtt_control.yaml
demos/common/config/mtt_path_follower.yaml
demos/common/config/mtt_repeat_supervisor.yaml
demos/common/config/wiln.yaml
```

Open a shell in the runtime image for debugging:

```bash
docker compose --profile debug run --rm bash
```

## Notes

- joystick teleop now runs directly inside the base `robot` service
- `teleop_robot` is kept only as a compatibility stub and should not be used as a second teleop stack
- `docker compose up` does not open Foxglove Studio for you. The bridge is just a websocket server.
- `rviz` is opt-in so the base robot stack stays lightweight.
- shared runtime behavior now lives in `../common/config/`
- Laptop-side monitoring, laptop-side teleop, and laptop-side bagging now live in `../monitor/`.
- The old scripts under `src/external/norlab_robot/scripts/user_scripts/` are still useful as references, but Compose is now the intended operator-facing entry point.
- If Compose warns about orphan containers after a service layout change, clean them once with:

```bash
docker compose down --remove-orphans
```

## Teach And Repeat

Start the base robot stack first:

```bash
docker compose up robot
```

Start WILN in a second terminal:

```bash
docker compose --profile wiln up wiln
```

Fresh route in the same live session:

```bash
docker compose run --rm wiln_teach_start
# drive manually
docker compose run --rm wiln_teach_stop
docker compose run --rm wiln_save
docker compose run --rm wiln_validate
docker compose run --rm wiln_replay
```

Save is optional if you want to reuse the route later:

```bash
docker compose run --rm wiln_save
```

Replay an existing saved route:

```bash
docker compose run --rm wiln_load
docker compose run --rm wiln_validate
docker compose run --rm wiln_replay
```

Diagnostics:

```bash
docker compose run --rm wiln_status
docker compose run --rm wiln_validate
```

Before replaying a saved route on the real robot, make a preview plot from the
workspace root:

```bash
python3 scripts/preview_wiln_route.py data/route.ltr
```

If the preview reports high curvature or steering saturation, replay slowly and
lower the follower gains before hardware testing.
