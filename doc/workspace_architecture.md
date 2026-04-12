# MTT workspace architecture

## Goal

`mtt_tools` is the MTT core, not the whole robot image.

The workspace is meant to stay small and honest:
- MTT-specific code lives here,
- robot integration stays in an external overlay,
- external drivers and reusable algorithms are imported instead of copied,
- local demos stay separate from the core packages.

That keeps the repository maintainable while still allowing a full robot workspace to be rebuilt from GitHub.

## Target layout

```text
mtt_tools/
  src/
    mtt_*/
    external/
      norlab_robot/
      ...
  dependencies/
    robot.repos
  demos/
  doc/
  documentations/
  scripts/
```

## Layers and responsibilities

### `src/mtt_*`

This is the MTT core.

It should contain:
- the MTT CAN driver and ROS wrapper,
- MTT-specific messages and services,
- the MTT base description,
- MTT-specific bringup and simulation helpers that genuinely belong to the vehicle.

It should not absorb:
- generic sensor drivers,
- generic mapping or controller packages,
- robot-side startup logic for every deployment,
- project-specific operator glue that is not MTT-specific.

### `src/external/norlab_robot`

This is the runtime integration layer for the real robot.

In the current observed stack, this is where the robot-wide assembly happens:
- startup,
- sensors,
- mapping,
- recording,
- Foxglove,
- Zenoh,
- operator workflows.

Keeping it external is intentional. It lets `mtt_tools` remain a vehicle repository instead of turning into a monolithic lab workspace.

### Other `src/external/*`

These are imported dependencies:
- drivers,
- estimation packages,
- controller hooks,
- shared Norlab interfaces.

They should be pulled in through `vcstool` and versioned by manifest, not copied into the MTT repository.

### `dependencies/*.repos`

This is the source of truth for the external workspace composition.

For MTT, the default file is `dependencies/robot.repos`.
It should stay curated:
- include what is needed for the observed MTT runtime,
- keep uncertain or unverified dependencies out until they are confirmed,
- avoid turning the manifest into a dump of every historical Norlab repo.

### `demos/`

`demos/` contains runnable entry points and operator-side workflows.

In the current repository, it is the right place for:
- local monitoring,
- laptop-side teleoperation,
- simulation wrappers,
- recording helpers used around the core packages.

If a future runtime overlay grows beyond demo scope, add a dedicated `deployments/` directory later rather than pushing more deployment logic into `src/mtt_*`.

### `doc/` and `documentations/`

- `doc/` is for repository documentation, conventions, and workflow notes.
- `documentations/` is for reference material: manuals, specifications, DBC files, and static notes that do not drive the build.

## Bootstrap flow

The bootstrap stays intentionally small:

1. `./scripts/create_env`
   prepares the local `.env` and bind-mounted host paths used by the development tools.
2. `./scripts/create_ws`
   imports the external repositories listed in `dependencies/robot.repos` into `src/external/`.
3. `colcon build --base-paths src src/external --symlink-install`
   builds the full workspace.

This follows the useful part of the T-Rex pattern:
- explicit manifest,
- explicit bootstrap,
- stateless workspace assembly,
- no git submodule sprawl.

## Namespace conventions

The repository is not fully there yet, but the target convention is straightforward.

### Launch API

- top-level launch files should expose `robot_namespace`,
- `use_namespace` can stay as the on/off switch when needed,
- when including third-party launch files that expect `namespace`, adapt at the boundary instead of spreading both names everywhere.

### Topics and services

- use relative names in code and YAML by default,
- avoid leading `/` in package defaults,
- keep absolute remaps only for ROS exceptions and third-party interoperability such as `/tf` and `/tf_static`.

### TF and frames

- do not hardcode world or robot frames in code,
- use parameters such as `base_frame`, `odom_frame`, and `map_frame`,
- let the runtime integration layer decide the final names and prefixes.

### Config split

- vehicle defaults belong with the MTT packages,
- robot-instance and deployment-specific config belongs in the runtime overlay or the demo/deployment directory,
- site or mission overrides should stay outside the vehicle core.

## Path to broader reuse

This structure already prepares the project for future variants:
- another MTT configuration can reuse the same `mtt_*` packages with a different integration overlay,
- another robot can reuse the external drivers and algorithms without inheriting the MTT CAN layer,
- future controller or mapping work can move toward generic packages without forcing a rewrite of the MTT core.

The next meaningful refactor is not to merge everything into `mtt_tools`.
It is to make the runtime boundary cleaner and the namespace handling consistent.

## Known gaps

- OAK support is still incomplete at the workspace level. The launch file exists in `norlab_robot`, but the exact `depthai_ros_driver` dependency still needs to be confirmed.
- The full startup truth still lives on the robot side and needs a focused runtime validation pass.
- Several older launch files still carry pre-existing namespace and absolute-topic habits that should be cleaned up in a dedicated follow-up pass.
