# MTT workspace

ROS 2 Jazzy workspace for the MTT-154 robot.

For the internship handover, start with the [handover guide](docs/reference/handover.md)
and the [current audit](docs/reference/robotics_safety_audit_2026-09-26.md).
For datasets and model evaluation, use the [research guide](scripts/research/README.md).

Use Docker unless you have a specific reason not to. The Docker path installs
the system dependencies, imports every repository, builds the workspace and
keeps the host clean.

## Safety first

The live stacks can initialize CAN, sensors, joystick control and autonomous
replay. On a real robot:

- lift or mechanically secure the tracks before the first test;
- keep the physical emergency stop reachable;
- do not run `demos/live_robot`, `demos/data_collection`, `constant_speed`,
  `mtt_brake_cycle.py`, `test_articulation.py` or `test_cl86ec.py` unless you
  understand what they command;
- use bag replay or simulation for the first verification.

Building the workspace does not contact the robot.

## New machine: three commands

Requirements:

- Linux x86_64;
- Git access to the private `norlab-ulaval` repositories;
- Docker Engine with the Compose plugin;
- Python 3 with PyYAML for workspace verification (`python3-yaml` on Ubuntu);
- at least 50 GB free disk space for images and build files.

Clone and build:

```bash
git clone git@github.com:norlab-ulaval/mtt_workspace.git
cd mtt_workspace
./scripts/setup
```

`setup` creates the local `.env`, initializes all recursive submodules, builds
the Docker images when needed, compiles ROS and runs the workspace doctor.
It never starts a robot service.

Build the optional Isaac ROS sidecar too:

```bash
./scripts/setup --with-isaac
```

Check the result at any time:

```bash
./scripts/status --doctor --summary
```

Offline verification after any edit:

```bash
./scripts/verify
```

This checks source completeness, syntax, documentation, Compose and tooling
regressions. It allows local edits; `./scripts/verify --release` additionally
requires clean repositories. Run `./scripts/test_offline` for an isolated fresh
build and behavioral tests of the principal MTT control/driver/perception packages.

Open a development shell:

```bash
docker compose run --rm bash
```

## First run without a robot

Replay a bag:

```bash
./scripts/replay_bag.sh /absolute/path/to/session_or_bag
```

Then connect Foxglove Studio to `ws://localhost:8765`.

For simulation:

```bash
cd demos/simulation
docker compose up simulation control rviz
```

These are the normal first checks. Do not use the live demo as an installation
test.

## Real robot

Configure the target once:

```bash
./scripts/create_env --robot-target robot@192.168.2.2
```

Copy the workspace to the robot:

```bash
./scripts/autosync_ws
```

On the robot, build first, read the live checklist, then start only the service
you need:

```bash
./scripts/compile
cd demos/live_robot
docker compose up
```

Full instructions: [live robot README](./demos/live_robot/README.md).

## What is where

```text
mtt_workspace
├── src/mtt_core                  MTT-owned ROS packages
├── src/external                  imported ROS repositories
├── dependencies/robot.repos      vcstool mirror of .gitmodules
├── docker                        image definitions and ROS entrypoint
├── demos                         operator-facing Compose stacks
├── scripts                       setup, checks, field tools and research tools
├── docs/reference                current operational references
└── documentations                manuals, CAN files and historical analyses
```

The parent repository owns Docker, Compose, dependency pins, helper scripts and
workspace documentation. Runtime ROS code lives mainly in:

- `src/mtt_core`: driver, control, description, localization, perception and
  MTT interfaces;
- `src/external/norlab_robot`: sensor and runtime integration;
- `src/external/wiln`: teach-and-repeat;
- `src/external/norlab_icp_mapper_ros`: ICP mapping;
- the remaining `src/external/*` directories: third-party drivers and libraries.

`.gitmodules` is the source of truth. `dependencies/robot.repos` is the vcstool
mirror. A parent commit pins exact submodule commits, so a branch name alone is
not enough to reproduce a deployment.

## Runtime stacks

| Directory | Runs where | Purpose | Hardware risk |
| --- | --- | --- | --- |
| `demos/bag_replay` | workstation | MCAP replay, mapping and diagnostics | none when no live ROS graph is connected |
| `demos/simulation` | workstation | Gazebo and controller tuning | none when isolated from the robot network |
| `demos/monitor` | operator laptop | Zenoh client, Foxglove, optional remote joystick | command-capable with manual/command profiles |
| `demos/live_robot` | robot | driver, sensors, mapping and teleop | high |
| `demos/data_collection` | robot | live stack plus curated recording | high |
| `demos/mathis_com_shift` | robot/replay | COM motor and mapping experiments | high on hardware profiles |

See [demos/README.md](./demos/README.md) for the exact commands and profiles.

## Configuration rule

Normal runtime tuning belongs in:

```text
demos/common/config/
demos/live_robot/config/runtime.env
demos/data_collection/config/runtime.env
demos/mathis_com_shift/config/runtime.env
```

Do not tune package defaults under `src/**/config` first. The demo files are the
operator surface and may override package defaults.

## Command path

```text
joystick -> cmd_vel/manual_raw -> cmd_vel/manual ----+
                                                    +-> mtt_cmd_arbiter_node -> cmd_vel -> mtt_can_node -> CAN
WILN/path follower -> controller/cmd_vel -----------+
estop, mode and timeouts ----------------------------^
```

Only `mtt_cmd_arbiter_node` should publish the final `cmd_vel`. The driver has
command and telemetry timeouts, but those are not a replacement for the
physical emergency stop.

In the committed driver configuration, `linear.x` is m/s and `angular.z` is
**normalized steering in [-1, 1]** (`cmd_angular_mode: normalized_steer`).
Yaw rate in rad/s applies only when `cmd_angular_mode: yaw_rate` is selected
consistently along the command chain. See the [calibration and geometry reference](docs/reference/calibration.md).

## Network and Zenoh

Normal lab defaults:

| Use | Robot address | Port |
| --- | --- | --- |
| SSH and robot LAN | `192.168.2.2` | `22/tcp` |
| Zenoh router | `192.168.2.2` or `192.168.50.2` in Doodle mode | `7447/tcp` |
| robot Foxglove bridge | same robot address | `8765/tcp` |
| local monitor bridge | operator computer | `8766/tcp` |

Sensor Ethernet interfaces must never own the default route. Doodle uses
`192.168.50.0/24` and should also have no default route unless Internet sharing
is being configured intentionally.

Read these before field work:

- [networking reference](./docs/reference/networking.md);
- [Zenoh reference](./docs/reference/zenoh.md);
- [Doodle field handoff](./docs/reference/doodle_field_handoff.md).

## Data and research files

ROS bags, maps, generated figures, scratch analyses, caches and paper results
stay local and are ignored by Git. They belong under `data/`, `artifacts/`,
`results/`, `.scratch_test/` or another explicit output directory.

Operational scripts and research scripts share `scripts/` for historical
reasons. The separation, risk level and naming rule are documented in
[scripts/README.md](./scripts/README.md).

## Common commands

```bash
./scripts/setup                    # complete local setup, no robot startup
./scripts/create_ws                # .env and repositories only
./scripts/compile                  # Docker image check and ROS build
./scripts/status --doctor --summary
./scripts/verify                    # syntax, Compose and structure checks
./scripts/pull                     # fast-forward parent, restore exact submodule pins
./scripts/test_offline             # isolated build and behavioral tests; existing Docker image
./scripts/autosync_ws --dry-run    # preview a robot sync
```

## Troubleshooting

Docker permission denied:

```bash
sudo usermod -aG docker "$USER"
```

Log out and back in, then run `docker info`.

Submodule error, empty repository or `not our ref`:

```bash
git submodule sync --recursive
git submodule update --init --recursive
./scripts/status --doctor --summary
```

Generated files owned by another user:

```bash
sudo chown -R "$(id -u):$(id -g)" build install log .ccache
```

More details: [operations handoff](./docs/reference/operations.md).

To add packages, repositories, services or settings, use the
[developer guide](./docs/reference/development.md).

## Before freezing or handing over

```bash
./scripts/compile
./scripts/status --doctor --summary
git submodule status --recursive
```

Every modified nested repository must be committed and pushed before the parent
submodule pointer is committed. A clean parent with dirty nested repositories is
not a reproducible release.

Keep a local source snapshot before repository access ends:

```bash
python3 scripts/handover_snapshot.py --output artifacts/handover/source-snapshot
```

The destination must be new. This captures source files, local research helpers,
checksums and per-repository patches. Bags, images, Git history and credentials
need their own storage plan; see the handover guide.
