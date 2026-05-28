# data_collection

This demo is for robot-side data capture sessions.

It keeps three things together:
- the live robot stack,
- the session metadata,
- the bag recording workflow.

## Usual flow

Run the pre-checks first:

```bash
bash scripts/pre_session_check.sh
```

Start a recording session from `demos/data_collection`:

```bash
SESSION_TYPE=nominal \
TRAILER_ATTACHED=false \
OPERATOR=MA \
GPS_MODE=serial \
docker compose --profile record up
```

That starts:
- `robot`
- `socketcan_bridge`
- `record`

If you also want Compose to own the robot-side Zenoh router and Foxglove bridge:

```bash
docker compose --profile record --profile infra up
```

Stop with `Ctrl-C`. The bag is finalized cleanly on SIGINT.

## Runtime tuning

Edit the demo-owned config files first when you want to change robot behavior:

```text
demos/data_collection/config/runtime.env
demos/common/config/mtt_driver.yaml
demos/common/config/mtt_control.yaml
demos/common/config/mtt_path_follower.yaml
demos/common/config/mtt_repeat_supervisor.yaml
demos/common/config/wiln.yaml
```

Use them like this:
- `runtime.env`: demo toggles and launch-level switches
- `demos/common/config/mtt_driver.yaml`: final drive scaling and articulation model
- `demos/common/config/mtt_control.yaml`: joystick mapping, deadman, filters, and final arbiter behavior
- `demos/common/config/mtt_path_follower.yaml`: follower speed and path-tracking behavior
- `demos/common/config/mtt_repeat_supervisor.yaml`: replay gating, override, and cancel behavior
- `demos/common/config/wiln.yaml`: WILN route replay speed and save/load settings

For replay tuning, change these first:
- `trajectory_speed` in `wiln.yaml`
- `default_speed_ms`, `max_speed_ms`, `min_speed_ms` in `mtt_path_follower.yaml`

If replay publishes `linear.x` but the robot still does not pull hard enough on terrain, only then adjust `max_linear_speed_ms` in `mtt_driver.yaml`.

## Useful profiles

- `--profile record`
  start the bag recorder
- `--profile infra`
  start the robot-side Zenoh router and Foxglove bridge
- `--profile check`
  run the health check helper
- `--profile viz`
  start RViz against the live robot TF tree
- `--profile debug`
  open a shell in the runtime image

`teleop` still exists as a compatibility stub, but joystick teleop now runs directly inside the base `robot` service.

## GPS mode

Use USB by default:

```bash
GPS_MODE=serial docker compose --profile record up
```

Use TCP only when the antenna setup requires it:

```bash
GPS_MODE=tcp docker compose --profile record up
```

## Output layout

Each session is stored under `data/`:

```text
data/mtt_<session_type>_<experiment>_<timestamp>/
  bag/
  session_info.yaml
  ros_params.yaml
  topic_list.txt
  topic_list_with_types.txt
```

The topic list comes from `src/external/norlab_robot/config/rosbag_record/all_sensors_full.yaml`.

The recorder does not use `--all`.

That is intentional:
- the live stack exposes late publishers and transient-local topics such as `/tf_static` and `/robot_description`
- `--all` also pulls in image transport side topics we do not want, which can trigger noisy camera-side transport errors

The data-collection flow records the curated topic list with explicit QoS overrides instead.

That curated list now also includes:
- ZED registered depth
- ZED registered point cloud
- OAK aligned depth on `/oak/stereo/image_raw`
- OAK RGBD point cloud on `/oak/points`

Open RViz from the same live stack when needed:

```bash
docker compose --profile viz up rviz
```

## Teach And Repeat

Start the base robot stack first:

```bash
docker compose up robot
```

The field profile records the live ICP map topic and the raw Hesai/RS-Airy/ZED/OAK streams needed for replay. It does not start or record `/merged_points*` by default; mapping uses `/hesai_lidar/points`.

Start WILN in a second terminal:

```bash
docker compose --profile wiln up wiln
```

Do not use `ros2 launch wiln` directly for the MTT stack here. The WILN package still ships generic legacy launch files; the MTT entry point is `teach_repeat.launch.py` through `norlab_robot` or the Compose service above.

Fresh route in the same live session:

```bash
docker compose run --rm wiln_teach_start
# drive manually
docker compose run --rm wiln_teach_stop
docker compose run --rm wiln_save
docker compose run --rm route_check
docker compose run --rm route_replay
```

When `ROUTE` is not set, `wiln_save` creates a timestamped route under
`data/wiln_routes/` and updates `data/wiln_routes/latest`. To name a route
explicitly:

```bash
ROUTE=garage_1559 docker compose run --rm wiln_teach_start
# drive manually
ROUTE=garage_1559 docker compose run --rm wiln_teach_stop
ROUTE=garage_1559 docker compose run --rm wiln_save
ROUTE=garage_1559 docker compose run --rm route_validate
ROUTE=garage_1559 docker compose run --rm route_preview
ROUTE=garage_1559 docker compose run --rm route_check
```

Replay an existing saved route:

```bash
docker compose run --rm route_list
docker compose run --rm route_load      # uses latest if ROUTE is not set
docker compose run --rm route_replay
```

Diagnostic helper:

```bash
docker compose run --rm wiln_status
docker compose run --rm field_ready
docker compose run --rm icp_check
```

Important:
- you do not need a saved map file before `teach_start`
- you do need live `/mapping/icp_odom`
- `icp_check` must pass before autonomous repeat
- `teach_stop` already arms the route in memory, so `load` is only for replaying an older saved route

## Command chain

The current command chain is:

```text
joy -> cmd_vel/manual_raw -> cmd_vel/manual
WILN (/wiln/command + /wiln/trajectory) -> /wiln/control/local_plan -> controller/cmd_vel
mode manager + arbiter -> cmd_vel
driver -> mtt_can_node
```

Only `mtt_cmd_arbiter_node` should publish the final `cmd_vel` consumed by the MTT driver.

The demo-owned control config comes from:

```text
demos/common/config/mtt_control.yaml
```

Operator mode rules:
- `A` requests `AUTO`
- `Y` requests `MANUAL`
- `B` requests `STOP`
- any real joystick motion with deadman held forces `MANUAL` and the supervisor cancels replay

## Safety behavior

| State | Input condition | Final command behavior | Why |
| --- | --- | --- | --- |
| E-stop | `teleop_estop=true` | `mtt_can_node` forces neutral command and safety lock | stop has priority over everything |
| Manual idle | deadman released | teleop stops publishing active intent | autonomy must not be masked by stale manual zeros |
| Manual control | deadman held and stick moved | `cmd_vel/manual` wins in the arbiter | operator priority |
| Autonomous replay | follower active, no manual override | `controller/cmd_vel` wins in the arbiter | single final command path |
| Source timeout | `cmd_vel` becomes stale | driver neutralizes throttle and steer | no stale motion |

## Replay debug

When heading is corrected but the robot still does not pull forward, check the chain in this order:

```bash
ros2 topic info /controller/cmd_vel
ros2 topic info /cmd_vel
ros2 topic hz /mapping/icp_odom
ros2 topic echo /controller/cmd_vel --once
ros2 topic echo /cmd_vel --once
ros2 param dump /mtt_can_node
```

Interpretation:
- `/controller/cmd_vel` non-zero, `/cmd_vel` zero: mode selection or arbiter gating is blocking autonomy
- `/cmd_vel` non-zero but chassis does not move: the issue is downstream of ROS, usually throttle scaling, traction, brake, or hardware
- `/mapping/icp_odom` stale: replay will stop cleanly by design
