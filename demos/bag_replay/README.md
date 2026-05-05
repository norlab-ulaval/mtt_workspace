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
- `joint_state_converter`
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
- `joint_state_converter` rebuilds `joint_states` from recorded tachometer and
  `cmd_vel` so the current URDF still animates on older bags.
- the replay-side articulation mapping is configured directly in `compose.yaml`
  with `runtime_joint_pitch_rad`, `runtime_joint_roll_rad`, `runtime_joint_articulation_sign`,
  and `runtime_joint_articulation_offset_rad`
- with the current MTT URDF, replay uses `runtime_joint_pitch_rad = -pi/2` and
  `runtime_joint_roll_rad = -pi/2`, plus a `runtime_joint_articulation_offset_rad = +pi/2`,
  so the trailer chain matches the simulation-style rest pose and points toward the detected trailer frame instead of staying rotated by 90 degrees
- replay excludes recorded `/tf`, `/tf_static`, and recorded joint-state topics
  by default so the current URDF and replay-side odometry remain the only body-state owners
- the replay-side driver, localization, perception, and RViz now all consume bag time
- live bags do not need to record `/clock`; replay synthesizes it via `ros2 bag play --clock`
- If the bag comes from `demos/data_collection`, the session metadata is printed at startup.
- Raw sensor packets stay useful here because replay is not limited to the already-processed topics.
- `offline_icp` skips sessions that already have both `map.vtk` and `trajectory.vtk`; set `OFFLINE_ICP_FORCE=true` to rebuild them.
