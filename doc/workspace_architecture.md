# MTT workspace architecture

## Goal

Keep the workspace honest and easy to rebuild.

The current model is:
- `mtt_workspace`
  parent repo for infra, docs, scripts, and manifests
- `src/mtt_core`
  imported MTT-owned ROS packages
- `src/external`
  imported dependencies
- `src/external/norlab_robot`
  runtime integration overlay for the real robot

## Layout

```text
mtt_workspace/
  src/
    mtt_core/
    external/
      norlab_robot/
      ...
  dependencies/
    robot.repos
  docker/
  demos/
  doc/
  documentations/
  scripts/
  data/
```

## Responsibilities

### Parent repo: `mtt_workspace`

Owns:
- Docker and devcontainer files
- dependency manifests
- local demos
- helper scripts
- project docs

Does not own:
- all robot ROS code
- the runtime overlay
- copied external dependencies

### Core repo: `src/mtt_core`

Owns:
- MTT-specific driver code
- MTT description
- MTT bringup
- MTT interfaces and messages

This is the right place for MTT-owned ROS packages.

### Imported dependencies: `src/external`

Owns:
- external drivers
- mapping and estimation packages
- shared Norlab repos
- runtime integration repos

These repos should stay imported, not copied.

### Runtime overlay: `src/external/norlab_robot`

Owns:
- sensor composition
- startup glue
- mapping launch composition
- Foxglove and Zenoh runtime wiring
- robot-side recording and utilities

If runtime truth is wrong there, fix it there.

## Bootstrap flow

The intended flow is small and explicit:

1. `./scripts/create_env`
   creates `.env` and local bind-mount directories
2. `./scripts/create_ws`
   imports `src/mtt_core` and the repos listed in `dependencies/robot.repos`
3. `colcon build --base-paths src/mtt_core src/external --symlink-install`
   builds the current workspace composition

## Nested repos

The parent repo does not track the contents of `src/mtt_core` or `src/external`.

That means:
- root-level `git status` is not enough
- root-level `git pull` is not enough

Use:
- `./scripts/status`
- `./scripts/pull`

## Namespace direction

The target model is simple:
- top-level launch files expose `robot_namespace` and `use_namespace`
- topics and services are relative by default
- frames are parameterized
- namespace adaptation happens at the launch boundary

The cleanup is not finished yet, but this is the contract to keep extending.

## What this structure buys us

- the parent repo stays small enough to understand
- `mtt_core` can evolve as the owned vehicle stack
- `norlab_robot` stays where runtime assembly actually happens
- imported dependencies remain explicit and reproducible
- future robot variants can reuse the same pattern without turning this repo into a dump of everything
