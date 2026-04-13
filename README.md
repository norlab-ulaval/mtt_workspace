# MTT_workspace

![MTT Robot](./mtt.jpg)

ROS 2 Jazzy workspace for the MTT platform.

This repository is the overarching workspace foundation. Once cloned and bootstrapped, it automatically imports the core MTT packages (`mtt_core`) along with all external sensor drivers and hardware dependencies necessary to run the robot.

It contains:
- the local workspace infrastructure used for development,
- the dependency manifests needed to reconstruct the full software stack,
- operator-side demos and reference documentation for the MTT.

The real robot stack is modular. The current runtime observed on the robot depends on:
- `mtt_core` for the fundamental TF trees, custom drivers, and descriptions.
- `norlab_robot` as the runtime integration layer,
- external drivers and estimation/control packages.

The goal of this repository is to decouple the workspace environment (Docker, scripts, network bridges) from the actual ROS 2 source code logic.

## Repository layout

```text
mtt_workspace/
  src/
    mtt_core/            # imported with vcs (contains Bringup, Description, Driver, Msgs)
    external/            # imported with vcs (contains hardware drivers, norlab_robot)
  dependencies/
    robot.repos          # external repos needed for the complete robot stack
  docker/
  demos/
  doc/
  documentations/
  scripts/
```

## Workspace layers

- `src/mtt_core`
  The MTT-specific ROS 2 packages dynamically cloned into the workspace. It contains the primary algorithms and implementations.
- `src/external/*`
  External repositories imported with `vcstool`. They stay outside the parent git history.
- `src/external/norlab_robot`
  Runtime integration layer for the real robot. This is where the observed sensor, mapping, startup, and recording orchestration currently lives.
- `dependencies/robot.repos`
  Curated manifest for all external dependencies including `mtt_core`.
- `demos/`
  Local entry points and operator-side workflows.
- `doc/`
  Repository documentation and engineering conventions.
- `documentations/`
  Reference material such as specifications, manuals, and DBC files.

## Quick start

### Full robot workspace bootstrap

Use this when setting up a new machine to acquire the full run-time stack.

Requirements:
- `vcstool` installed and available as `vcs`,
- SSH access to the private repositories listed in `dependencies/robot.repos`,
- ROS 2 Jazzy already installed on the machine.

```bash
./scripts/create_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/mtt_core src/external --symlink-install
source install/setup.bash
```

### Docker workflow

The Docker files in this repository support local development inherently matching the physical hardware dependencies.

```bash
./scripts/create_env
docker compose -f docker/build.yaml build devel
docker compose run --rm compile
docker compose run --rm bash
```

The Docker build logic now detects `src/mtt_core` and `src/external/` automatically. It will mount and compile all imported packages. Rebuild the image (`docker compose -f docker/build.yaml build devel`) whenever a new repository is added to the manifest.

## Utilities and Scripts

The `scripts/` directory provides workflow automation tools:

- `./scripts/create_ws`
  Reads `dependencies/robot.repos` and utilizes `vcs` to clone all missing packages (including `mtt_core`) into `src/`.
- `./scripts/create_env`
  Generates the mandatory `.env` file initializing network targets, user IDs, and Git configurations necessary to route data cleanly between Docker and the host.
- `./scripts/pull`
  Recursively performs a fast `git pull` on all inner repositories inside your workspace, guaranteeing everything is up to date.
- `./scripts/autosync_ws`
  Deploys your locally compiled binaries and parameters to the designated real robot IP using `rsync`, circumventing the need to physically clone code directly on the MTT onboard computer.
- `./scripts/status`
  Runs a comprehensive scan across all nested submodules ensuring no uncommitted files are omitted before deployment.

### Examples

```bash
./scripts/create_env --robot-target mohamed@192.168.2.2
./scripts/create_ws
./scripts/autosync_ws
docker compose --env-file .env -f demos/live_robot/compose.yaml up monitor
```

