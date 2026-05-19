# bag_replay

This demo replays a recorded MCAP bag against the MTT stack with bag time.

## Basic use

From `demos/bag_replay`:

```bash
BAG_PATH=/path/to/data/mtt_nominal_2026-04-15_15-41-53 docker compose up
```

Or use the helper from the workspace root:

```bash
./scripts/replay_bag.sh /path/to/session_or_bag
./scripts/replay_bag.sh /path/to/session/session.mcap up rviz
```

That starts:
- `bag_player`
- `description`
- `runtime_odometry`
- `joint_state_builder`
- `foxglove`
- `rviz` can be started separately if you want the same calibrated robot model there too

Open Foxglove Studio separately and connect to `ws://localhost:8765`.

Open RViz against the replay stack:

```bash
BAG_PATH=/path/to/bag docker compose up rviz
```

## Optional profiles

Replay with localization:

```bash
BAG_PATH=/path/to/bag docker compose --profile localization up
```

Replay with perception:

```bash
BAG_PATH=/path/to/bag docker compose --profile perception up
```

Rebuild one session offline with ICP mapping:

```bash
BAG_PATH=/path/to/session docker compose --profile offline_icp up offline_icp
```

Validate WILN teach-and-repeat on a clean ICP bag:

```bash
BAG_PATH=/path/to/session REPLAY_RATE=0.5 docker compose --profile mapping --profile teach_repeat up bag_player description perception mapping foxglove teach_repeat
```

During replay, call WILN services from another shell in the same Compose project:

```bash
docker compose --profile debug run --rm bash
ros2 service call /start_recording std_srvs/srv/Empty "{}"
ros2 service call /stop_recording std_srvs/srv/Empty "{}"
ros2 service call /save_map_traj wiln/srv/SaveMapTraj "{file_name: {data: '${WORKSPACE}/data/route.ltr'}}"
python3 ${WORKSPACE}/scripts/validate_wiln_route.py ${WORKSPACE}/data/route.ltr
ros2 service call /load_map_traj wiln/srv/LoadMapTraj "{file_name: {data: '${WORKSPACE}/data/route.ltr'}}"
ros2 service call /play_line std_srvs/srv/Empty "{}"
```

The route is usable only if `/mapping/icp_odom` is continuous and the route validator does not reject the `.ltr`.

Score the existing bag results before choosing a WILN test route:

```bash
python3 ${WORKSPACE}/scripts/evaluate_wiln_readiness.py ${WORKSPACE}/data
cat ${WORKSPACE}/data/wiln_readiness_summary.csv
```

Export a WILN route directly from a trusted ICP CSV:

```bash
SESSION=${WORKSPACE}/data/mtt_calibration_test_garage_2026-04-29_15-59-38
python3 ${WORKSPACE}/scripts/export_icp_route_to_ltr.py \
  ${SESSION}/motion_model_validation/model_dataset.csv \
  ${SESSION}/wiln_routes/icp_route.ltr
python3 ${WORKSPACE}/scripts/validate_wiln_route.py \
  ${SESSION}/wiln_routes/icp_route.ltr
python3 ${WORKSPACE}/scripts/preview_wiln_route.py \
  ${SESSION}/wiln_routes/icp_route.ltr \
  --dataset-csv ${SESSION}/motion_model_validation/model_dataset.csv
```

The preview writes:
- `wiln_routes/preview/wiln_route_preview.png`
- `wiln_routes/preview/wiln_route_preview.yaml`

Check the preview before any real replay. If it reports `steering_near_saturation`
or `high_curvature`, keep WILN slow and reduce follower gains before hardware.

Rebuild every `mtt_*` session under `data/`:

```bash
docker compose --profile offline_icp up offline_icp
```

Policy used by default:
- fuse Hesai + RS-Airy with `cloud_merger_node` when both raw LiDAR clouds exist
- fall back to the recorded merged cloud if present
- fall back to Hesai alone otherwise

Outputs written next to each session bag:
- `map.vtk`
- `trajectory.vtk`
- `offline_icp/summary.yaml`
- `offline_icp/logs/*.log`

Slow the replay down when debugging:

```bash
BAG_PATH=/path/to/bag REPLAY_RATE=0.5 docker compose up
```

Audit one bag against the expected topic list:

```bash
python3 ./scripts/audit_bag_topics.py /path/to/session_or_bag
python3 ./scripts/audit_bag_topics.py /path/to/session/session.mcap --show-ok
```

## Notes

- `use_sim_time` is handled by the replay-side launch paths.
- `runtime_odometry` recomputes the current articulation and odometry from the
  replayed tachometer/cmd stream on replay-local topics.
- `joint_state_builder` rebuilds the articulated `joint_states` stream from the
  replay-local articulation plus the tachometer-derived wheel travel so the
  current URDF animates like the simulation stack, even on older bags that
  never recorded `/joint_states`.
- replay excludes recorded `/tf`, `/tf_static`, and recorded joint-state topics
  by default so the current URDF and replay-side odometry remain the only body-state owners
- the replay-side driver, localization, perception, and RViz now all consume bag time
- live bags do not need to record `/clock`; replay synthesizes it via `ros2 bag play --clock`
- If the bag comes from `demos/data_collection`, the session metadata is printed at startup.
- Raw sensor packets stay useful here because replay is not limited to the already-processed topics.
- `offline_icp` skips sessions that already have both `map.vtk` and `trajectory.vtk`; set `OFFLINE_ICP_FORCE=true` to rebuild them.
