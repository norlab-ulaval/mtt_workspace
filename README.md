# mtt_tools

ROS 2 Jazzy workspace for the MTT platform.

This repository is the MTT core. It is not the full robot image by itself.

It contains:
- the MTT-specific ROS packages versioned in this repository,
- the local workspace infrastructure used for development,
- operator-side demos and reference documentation for the MTT.

The real robot stack is larger. The current runtime observed on the robot also depends on:
- `norlab_robot` as the runtime integration layer,
- external drivers and estimation/control packages,
- robot-side startup and sensor configuration that do not belong in the MTT core.

The goal of this repository is to keep the MTT-specific code clean while making the full workspace reconstructible from GitHub.

## Repository layout

```text
mtt_tools/
  src/
    mtt_bringup/
    mtt_description/
    mtt_driver/
    mtt_interfaces/
    mtt_msgs/
    external/            # imported with vcs, ignored by this repo
  dependencies/
    robot.repos          # external repos needed for the observed robot stack
  docker/
  demos/
  doc/
  documentations/
  scripts/
```

## Workspace layers

- `src/mtt_*`
  MTT-specific packages that belong to this repository.
- `src/external/*`
  External repositories imported with `vcstool`. They stay outside the parent git history.
- `src/external/norlab_robot`
  Runtime integration layer for the real robot. This is where the observed sensor, mapping, startup, and recording orchestration currently lives.
- `dependencies/robot.repos`
  Curated manifest for the external repositories needed to rebuild the observed MTT robot workspace.
- `demos/`
  Local entry points and operator-side workflows. These are deployments or usage wrappers, not the canonical source of robot integration.
- `doc/`
  Repository documentation and engineering conventions.
- `documentations/`
  Reference material such as specifications, manuals, and DBC files.

## Quick start

### 1. Core-only build

Use this when you only need the packages versioned in this repository.

```bash
./scripts/create_env
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

### 2. Full robot workspace bootstrap

Use this when you want the broader runtime stack observed on the real robot.

Requirements:
- `vcstool` installed and available as `vcs`,
- SSH access to the private Norlab repositories listed in `dependencies/robot.repos`,
- ROS 2 Jazzy already installed on the machine.

```bash
./scripts/create_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src src/external --symlink-install
source install/setup.bash
```

`./scripts/create_ws` does three things:
- prepares the local `.env` through `./scripts/create_env`,
- creates `src/external/` if needed,
- imports the external repositories declared in `dependencies/robot.repos`.

### 3. Docker workflow

The Docker files in this repository are for local development around `mtt_tools`.

```bash
./scripts/create_env
docker compose -f docker/build.yaml build devel
docker compose run --rm compile
docker compose run --rm bash
```

This is useful for the MTT core and the local demos. Extending the Docker shortcuts to the imported robot workspace is a follow-up step; for now, the host-side bootstrap above is the honest path for the full external stack.

## Real robot notes

- The real robot runtime is not defined by `mtt_tools` alone.
- The current integration point is `norlab_robot`, imported into `src/external/`.
- Sensor drivers, mapping, and controller hooks are external dependencies and should stay that way.
- `mtt_tools` should keep the MTT-specific code, not absorb the entire robot stack.

## Useful commands

```bash
./scripts/create_env
./scripts/create_ws
./scripts/pull
./scripts/status
./scripts/autosync_ws
```

Examples:

```bash
./scripts/create_env --robot-target mohamed@192.168.2.2
./scripts/autosync_ws
docker compose --env-file .env -f demos/live_robot/compose.yaml up monitor
```

## Known limits

- `dependencies/robot.repos` is intentionally curated. It covers the external stack we observed around MTT, not every repo ever referenced by Norlab.
- OAK support is still incomplete at the workspace level. `norlab_robot` has an OAK launch file, but the exact `depthai_ros_driver` dependency still needs to be confirmed on the robot before adding it to the manifest.
- CAN truth, steering truth, and startup truth still require robot-side verification. This repository does not pretend those points are fully settled.
- Some older launch files still reflect pre-existing namespace conventions and will need a focused cleanup pass before the runtime becomes properly multi-robot friendly.
