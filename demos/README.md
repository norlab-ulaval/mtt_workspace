# Demos

Compose files are the operator entry points. Run commands from the demo
directory so the correct `.env`, relative paths and project name are used.

## Choose one

| Need | Directory | Normal command |
| --- | --- | --- |
| replay an MCAP bag | `bag_replay` | `BAG_PATH=/abs/path docker compose up` |
| test locally in Gazebo | `simulation` | `docker compose up simulation control rviz` |
| monitor a live robot from a laptop | `monitor` | `docker compose up monitor` |
| run the robot | `live_robot` | `docker compose up` |
| record a field session | `data_collection` | `docker compose --profile record up` |
| reproduce the COM-shift setup | `mathis_com_shift` | read its README first |

Bag replay and simulation are the first-run paths. The other stacks can connect
to or move hardware.

## Common rules

- Run `../../scripts/compile` after changing ROS source, launch files or package
  metadata.
- Run `docker compose config --quiet` after changing Compose or environment
  files.
- Use `docker compose down` when switching stacks on the same machine.
- Use `docker compose down --remove-orphans` after service names change.
- Do not run two robot stacks at once. They can duplicate TF, command, CAN and
  sensor publishers.
- Runtime tuning belongs in `common/config` and the selected demo `config`.

## Profiles

Profiles are opt-in services. List them without starting anything:

```bash
docker compose config --profiles
docker compose --profile '*' config --services
```

Common names:

| Profile | Purpose | Risk |
| --- | --- | --- |
| `debug` | shell in the configured image | low unless commands are then run manually |
| `check` | health and topic checks | read-only but joins the live ROS graph |
| `viz` | RViz or visualization tools | read-only but high bandwidth |
| `record` | bag recorder | disk and bandwidth load |
| `manual` | joystick/manual command tools | can move the robot |
| `wiln-ctrl` or `wiln` | teach-and-repeat controls | can start autonomous motion |
| `com` / `com_button` / `com_speed` | EtherCAT COM motor | can move hardware |
| `command` | direct command publisher | can move the robot |

Exact profile names vary by demo. Always use `config --profiles` on the file you
are about to run.

## Shared configuration

The normal tuning surface is:

```text
demos/common/config/mtt_driver.yaml
demos/common/config/mtt_control.yaml
demos/common/config/mtt_front_obstacle_monitor.yaml
demos/common/config/mtt_path_follower.yaml
demos/common/config/mtt_repeat_supervisor.yaml
demos/common/config/mtt_route_manager.yaml
demos/common/config/wiln.yaml
```

Demo-specific `runtime.env` files enable services and select devices. They can
change live behavior even when the shared YAML files are unchanged.

## Detailed guides

- [live robot](./live_robot/README.md)
- [data collection](./data_collection/README.md)
- [laptop monitor](./monitor/README.md)
- [bag replay](./bag_replay/README.md)
- [simulation](./simulation/README.md)
- [Mathis COM shift](./mathis_com_shift/README.md)
- [shared configuration](./common/README.md)
