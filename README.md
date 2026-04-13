# mtt_workspace

ROS 2 Jazzy workspace for the MTT platform.

This repository is the workspace shell around the robot software. It owns the Docker files, the dependency manifests, the helper scripts, the demos, and the project documentation.

It does not contain the whole robot source tree by itself.

The actual ROS code is split into nested repositories:
- `src/mtt_core`
  MTT-owned packages such as the driver, description, bringup, interfaces, and messages.
- `src/external/*`
  External dependencies imported with `vcstool`.
- `src/external/norlab_robot`
  Runtime integration layer for the real robot.

## Repository layout

```text
mtt_workspace/
  src/
    mtt_core/
    external/
  dependencies/
    robot.repos
  docker/
  demos/
  doc/
  documentations/
  scripts/
  data/
```

## What lives where

- `mtt_workspace`
  Workspace infrastructure, manifests, docs, and local workflows.
- `src/mtt_core`
  MTT-specific ROS packages.
- `src/external`
  Imported dependencies that stay outside the parent git history.
- `src/external/norlab_robot`
  Robot runtime overlay: sensors, startup, mapping, recording, Foxglove, and related runtime glue.
- `dependencies/robot.repos`
  Source of truth for the observed full workspace composition.

## Quick start

Requirements:
- ROS 2 Jazzy installed on the host
- `vcstool` installed as `vcs`
- access to the private repositories listed in `dependencies/robot.repos`

### Bootstrap the workspace

```bash
./scripts/create_ws
```

That imports `src/mtt_core` and the external repositories declared in `dependencies/robot.repos`.

### Build on the host

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/mtt_core src/external --symlink-install
source install/setup.bash
```

### Build in Docker

```bash
./scripts/create_env
docker compose -f docker/build.yaml build devel
docker compose run --rm compile
docker compose run --rm bash
```

## Important workflows

- `./scripts/create_env`
  Creates `.env` and local bind-mount directories used by Docker and demos.
- `./scripts/create_ws`
  Imports `mtt_core` and the declared external repositories.
- `./scripts/status`
  Shows the state of the parent repo, `src/mtt_core`, and imported repos under `src/external`.
- `./scripts/pull`
  Pulls the parent repo, `src/mtt_core`, and the imported external repos.
- `./scripts/autosync_ws`
  Syncs the local workspace to a robot target with `rsync`.

## Demos

The repository includes two distinct live-runtime entry points in `demos/`:
- `demos/live_robot`
  robot-side runtime stack started with Compose
- `demos/monitor`
  laptop-side operator stack for monitoring, teleop, and recording

It also includes:
- description and RViz checks,
- simulation launch wrappers.

See [`demos/README.md`](./demos/README.md) for the Compose entry points.

## Known limits

- The parent repo is not the full robot runtime by itself.
- `dependencies/robot.repos` reflects the observed workspace, but runtime truth still depends on what is actually present on the robot.
- OAK support is still not closed.
- Foxglove default policy still needs a final runtime decision in `norlab_robot`.
- CAN truth and steering truth still require robot-side validation.

## Practical note

Because the workspace is built from nested repositories, a plain `git status` at the root is not enough. Use `./scripts/status` before assuming the workspace is clean.
