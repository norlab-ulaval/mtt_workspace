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

## Useful profiles

- `--profile record`
  start the bag recorder
- `--profile infra`
  start the robot-side Zenoh router and Foxglove bridge
- `--profile manual`
  add robot-side joystick teleop
- `--profile check`
  run the health check helper
- `--profile debug`
  open a shell in the runtime image

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
