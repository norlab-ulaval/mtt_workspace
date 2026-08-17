# Scripts

This directory contains both operational tools and research tools. They are not
all safe to run on a connected robot.

Use the tables below. If a script is not listed, read its help and source before
running it.

## Workspace administration

These do not start the robot.

| Script | Purpose |
| --- | --- |
| `setup` | complete first setup: repos, Docker build, ROS build and doctor |
| `create_ws` | create `.env` and initialize recursive submodules |
| `create_env` | generate local user, image, robot and network variables |
| `compile` | build images if needed, then run `colcon build` in Docker |
| `status` | report parent and nested repository state; use `--doctor --summary` |
| `verify` | offline syntax, Compose and structure checks; `--release` also requires clean repos |
| `pull` | fast-forward the parent and nested repositories |
| `install_host.sh` | optional native Ubuntu setup; Docker is preferred |
| `workspace_source_paths` | source roots used by the build |

## Deployment and transfer

| Script | Effect | Normal safe use |
| --- | --- | --- |
| `autosync_ws` | copies the workspace to a configured robot over SSH/rsync | run `--dry-run` first |
| `sync_bags.sh` | copies recorded sessions from the robot | run `--list` or `--dry-run` first |
| `replay_bag.sh` | starts the local bag replay Compose stack | safe only when isolated from a live ROS graph |
| `annotate_session.sh` | writes session metadata | use only on the intended session directory |
| `event_mark.sh` | records a field event marker | verify the active session first |

## Network and field checks

Read [the network reference](../docs/reference/networking.md) before changing
routes or interfaces.

| Script | Mode |
| --- | --- |
| `audit_network_field.sh` | read-only snapshot of interfaces, routes, Docker and ROS rates |
| `field_doodle_check.sh` | Doodle reachability and bandwidth preflight |
| `field_ready_check.py` | live readiness report |
| `pre_session_check.sh` | live data-collection preflight |
| `post_launch_check.sh` | live post-launch diagnostics |
| `internet_via_usb` | changes forwarding, NAT and routes; dry-run first |
| `tune_network_for_wifi.sh` | changes kernel network settings; run on both hosts only when required |
| `run_with_zenoh_session.sh` | renders a Zenoh client config and runs the requested command |
| `autosync_ws` | selects normal LAN, Doodle, direct Ethernet or remote SSH path |

## CAN, EtherCAT and actuator tools

The following group can access hardware. Some scripts only monitor by default,
but the interface is still live. Keep the robot secured and the emergency stop
reachable.

Read-only or primarily diagnostic:

- `mtt_can_monitor.py`
- `mtt_can_audit.py`
- `mtt_can_export.py` when used on a saved log
- `mtt_health_monitor.py`
- `audit_control_safety.py`
- `test_cl86ec.py` only with a disconnected motor or verified dry-run setup

Command-capable or host-changing:

- `mtt_brake_cycle.py`
- `test_articulation.py`
- `configure_cl86ec_pdos.py`
- `test_cl86ec.py`
- `setup_udev_reach_rs.sh`
- `set_icp_prior.py` on a live graph
- `mtt_rear_obstacle_monitor.py` when connected to the command chain

Do not use a command-capable script as an installation test.

## Route, mapping and replay tools

These are normally offline when pointed at saved data:

- `audit_bag_topics.py`, `audit_bag_timing.py`, `check_bags.py`
- `audit_tf_chain.py`, `detect_tf_conflict.py`
- `check_icp_odom.py`, `audit_icp_health.py`, `icp_map_quality_check.py`
- `validate_wiln_route.py`, `preview_wiln_route.py`
- `export_icp_route_to_ltr.py`, `align_route_to_pose.py`
- `offline_reference.py`, `direct_offline_icp.py`, `run_mapping_dataset.py`
- `calibrate_static_bag.py`, `calib_hesai_scan.py`, `calib_zed_body_validate.py`

Some of the same scripts can subscribe or publish on a live ROS graph. A command
is only offline if all of its inputs are files and no live ROS middleware is
connected.

## Research tools

The following families exist for the ICRA work and dataset analysis. They are
not required to operate the MTT:

- `build_gt_*`, `gt_*`, `qualify_gt_*`;
- `build_*dataset*`, `extract_*`, `merge_*`, `load_*`;
- `evaluate_*`, `compare_*`, `fit_*`, `tune_*`;
- `plot_*`, `render_*`, `visualize_*`;
- `mtt_motion_model/*`;
- `analyze_ice_session.py`, `mtt_experiment_*`;
- one-off KISS-ICP, covariance, MSA and motion-model scripts.

Research outputs belong under `artifacts/`, `results/`, `data/` or
`.scratch_test/`. Do not add generated CSV, maps, figures or bag-derived files
to the workspace root.

For new work, use this rule:

```text
scripts/<name>             reusable operator or maintenance tool
scripts/research/<name>    paper or dataset-specific tool
artifacts/<run>/            generated result worth keeping locally
.scratch_test/<name>/       disposable experiment
```

Existing research scripts stay in place to avoid breaking old commands. New
ones should use `scripts/research/`.

## Before adding a script

- provide `--help`;
- use paths relative to the workspace or accept them as arguments;
- default to dry-run for route, network, sync and hardware changes;
- print the target host, interface, topic or output directory before acting;
- never silently publish a nonzero command;
- put generated files in an ignored output directory;
- add the script to this file if it is operational.
