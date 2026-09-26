# MTT developer guide

This guide is for someone who wants to add a package, dependency, Compose
service, configuration or script and then use it on the robot.

## One-time setup

```bash
git clone git@github.com:norlab-ulaval/mtt_workspace.git
cd mtt_workspace
./scripts/setup
```

Use the Docker shell for development:

```bash
docker compose run --rm bash
```

The host needs Git, Docker, Python 3 and PyYAML for verification. ROS and vendor dependencies stay in the
image.

## Decide where the change belongs

| Change | Location |
| --- | --- |
| driver, control, localization, perception, description, MTT message | `src/mtt_core` |
| sensor/runtime integration or recording | `src/external/norlab_robot` |
| WILN teach-and-repeat logic | `src/external/wiln` |
| ICP mapper implementation | `src/external/norlab_icp_mapper_ros` |
| operator config shared by demos | `demos/common/config` |
| one demo toggle or device path | `demos/<demo>/config` |
| image dependency | `docker/Dockerfile*` |
| operator workflow | `demos/<demo>/compose.yaml` |
| reusable maintenance command | `scripts/` |
| paper/dataset experiment | `scripts/research/` or a research repository |

Avoid copying the same fix into a package default, `norlab_robot` and a demo
config. Choose one owner and pass the value explicitly.

## Normal edit loop

Before editing:

```bash
./scripts/status --summary --dirty-only
```

After editing source:

```bash
./scripts/compile
```

Run all fast offline workspace checks:

```bash
./scripts/verify
```

After editing Compose:

```bash
docker compose -f demos/<demo>/compose.yaml config --quiet
docker compose -f demos/<demo>/compose.yaml --profile '*' config --services
```

After editing shell:

```bash
bash -n scripts/<script>
```

Build and run the maintained behavioral tests in a container without network
or host devices:

```bash
./scripts/test_offline
```

This uses an existing image and fresh build directories under
`artifacts/offline-tests`. See the [handover guide](handover.md) for coverage
and limits. Do not run a live launch file as a unit test.

## Add a ROS package

MTT-owned packages belong inside `src/mtt_core`, which is its own Git
repository.

```bash
cd src/mtt_core
git switch -c <short-branch>
ros2 pkg create <package> --build-type ament_cmake
```

Then:

1. add explicit dependencies to `package.xml` and `CMakeLists.txt`;
2. add tests that run without hardware;
3. add launch/config only when the package owns those defaults;
4. build with `./scripts/compile` from the workspace root;
5. commit and push `mtt_core`;
6. commit the new `src/mtt_core` pointer in the parent workspace.

Do not create a package directly under the parent `src/` directory.

## Add an external repository

Use a submodule and keep the vcstool mirror aligned.

From the workspace root:

```bash
git submodule add -b <branch> <url> src/external/<name>
```

Add the same path, URL and branch/version to `dependencies/robot.repos`.

Verify:

```bash
./scripts/status --doctor --summary
git submodule status --recursive
```

Rules:

- `.gitmodules` is canonical;
- `robot.repos` must mirror it;
- the parent pins a commit, not the branch head;
- the pinned commit must be pushed and readable by the next user;
- do not keep required third-party patches only as a dirty submodule.

If a third-party repository cannot be pushed, prefer a build argument, a
NorLab fork or an explicit maintained patch strategy. Do not rely on an
uncommitted local edit.

Package-specific build arguments belong in `dependencies/colcon.meta`. This is
used for the ROMEA install include directory so those public repositories stay
clean.

## Add a Docker dependency

Choose the layer carefully:

- `docker/Dockerfile.base`: large vendor/system dependency that rarely changes;
- `docker/Dockerfile`: ROS/Python/build dependency used by the workspace;
- `docker/Dockerfile.isaac`: Isaac/NVIDIA-specific dependency.

Pin versions when a moving upstream version can break the build. Keep apt and
pip caches in their existing layers. Do not put host kernel module installation
in a container and assume it configures the robot host.

Rebuild:

```bash
docker compose --profile build build base devel_image
./scripts/compile
```

## Add a Compose service

1. extend the appropriate base service;
2. use the demo `.env` and demo-owned configs;
3. put optional tools behind a profile;
4. use explicit `depends_on` only for real startup requirements;
5. add a timeout/readiness check for required topics or devices;
6. make shutdown behavior explicit;
7. document whether the service is read-only or command-capable;
8. validate with `docker compose config --quiet`.

Command, CAN, EtherCAT and actuator services must never be hidden behind a
generic name such as `test` or `tool`. The profile and README must state that
they can move hardware.

## Add runtime configuration

Use names with units:

```text
timeout_s
rate_hz
distance_m
speed_ms
angle_rad
yaw_rate_rad_s
```

Document:

- default;
- unit and frame;
- valid range;
- behavior when missing or invalid;
- whether restart is required;
- whether it changes safety or command behavior.

Shared operational tuning belongs in `demos/common/config`. A demo-specific
hardware path or enable flag belongs in that demo's `runtime.env`.

## Add a script

Reusable scripts go in `scripts/` and must:

- support `--help`;
- accept paths/targets as arguments;
- print the target before acting;
- default to dry-run for destructive, network, sync or hardware changes;
- return nonzero on failure;
- put output under an ignored data/artifact directory;
- be added to `scripts/README.md`.

Paper and dataset code goes under `scripts/research/` or a separate research
repository. Do not add another one-off Python file at the workspace root.

## Test levels

Run the lowest sufficient level first.

1. syntax and pure unit tests;
2. package test in isolated Docker;
3. full Docker build;
4. bag replay regression;
5. simulation;
6. live graph diagnostics;
7. secured low-speed hardware test.

At the handoff snapshot, the 66-package build is green but the complete legacy
test tree is not. Known failures are recorded in the
[robotics safety audit](./robotics_safety_audit_2026-08-17.md). A developer must
not treat a successful build as a fully green test suite.

For command-related changes, include at least:

- e-stop priority;
- deadman release;
- stale input timeout;
- manual/autonomous conflict;
- startup neutral;
- shutdown neutral;
- reconnect without replaying an old command.

## Use the change on the robot

First commit and push it. Then update the parent pointer and sync:

```bash
./scripts/autosync_ws --dry-run
./scripts/autosync_ws
```

On the robot:

```bash
./scripts/compile
cd demos/live_robot
docker compose config --quiet
```

Review the live checklist before `docker compose up`.

## Multi-repository commit order

1. commit and push each modified nested repository;
2. confirm its remote contains the commit;
3. stage the nested pointer in `mtt_workspace`;
4. run the workspace doctor;
5. commit and push the parent;
6. verify a clean clone.

Example:

```bash
git -C src/mtt_core add <files>
git -C src/mtt_core commit -m "short change"
git -C src/mtt_core push

git add src/mtt_core
git commit -m "update core"
git push
```

Never commit generated `build/`, `install/`, `log/`, bags, maps or research
outputs.

## Review checklist

```bash
./scripts/compile
./scripts/status --doctor --summary
git diff --check
git submodule status --recursive
```

Also verify:

- README command matches the actual Compose service/profile;
- topic type and frame are documented;
- no new default route or hardcoded host was introduced;
- no duplicate final command or TF publisher exists;
- every required nested commit is pushed;
- a nontechnical user can still run `./scripts/setup` and bag replay.
