# MTT operations handoff

This is the shortest complete path from a new computer to a reproducible MTT
workspace.

## Supported setup

- Ubuntu/Linux x86_64;
- Docker Engine and Docker Compose plugin;
- Git and SSH access to the private NorLab repositories;
- NVIDIA Container Toolkit only for GPU/Isaac/ZED acceleration;
- X11 only for RViz and other GUI tools.

macOS, Windows, ARM and Docker Desktop are not supported deployment targets.
The live stacks use host networking, Linux devices and privileged containers.

## Install Docker

Use the organization or Ubuntu Docker installation procedure. The required
result is:

```bash
docker info
docker compose version
```

Both commands must work without `sudo`.

If access is denied:

```bash
sudo usermod -aG docker "$USER"
```

Log out and back in.

## Clone everything

```bash
git clone git@github.com:norlab-ulaval/mtt_workspace.git
cd mtt_workspace
./scripts/setup
```

If Isaac VSLAM is required on this machine:

```bash
./scripts/setup --with-isaac
```

The clone needs access to every private URL in `.gitmodules`. `setup` does not
start a robot process.

If setup stops during repository import:

```bash
git submodule sync --recursive
git submodule update --init --recursive
git submodule status --recursive
```

Prefix meaning:

- no prefix: exact pinned commit;
- `-`: not initialized;
- `+`: checked-out commit differs from parent pin;
- `U`: merge conflict.

## Build

```bash
./scripts/compile
```

The script:

1. updates `.env` with the local UID/GID and workspace path;
2. checks Docker access;
3. builds `mtt_workspace:base` and `mtt_workspace:devel` when missing or built
   for another user;
4. runs `colcon build` over `src/mtt_core` and `src/external`.

The source checkout of Isaac ROS Visual SLAM is kept as a pinned reference but
is excluded from the normal image build. Isaac runs from
`mtt_workspace:isaac`. KISS-ICP is built from its source submodule so a clean
build does not depend on stale files in `install/`.

Generated `build/`, `install/`, `log/` and `.ccache/` directories stay local.

## Verify without hardware

```bash
./scripts/status --doctor --summary
docker compose -f compose.yaml config --quiet
docker compose -f demos/bag_replay/compose.yaml config --quiet
docker compose -f demos/simulation/compose.yaml config --quiet
```

Then replay a known bag or run simulation. Do not start the live stack merely
to prove installation.

## Configure a robot target

```bash
./scripts/create_env --robot-target robot@192.168.2.2
```

Main generated values:

```text
ROBOT_SSH_TARGET
ROBOT_WORKSPACE
ROBOT_ZENOH_ENDPOINT
ROBOT_FOXGLOVE_URL
LIVE_ROBOT_DOMAIN_ID
```

`.env` is machine-local and ignored by Git.

## Transfer

Preview first:

```bash
./scripts/autosync_ws --dry-run
```

Normal LAN:

```bash
./scripts/autosync_ws
```

Doodle:

```bash
./scripts/autosync_ws --doodle
```

Direct Ethernet and private remote targets are available through `--eth` and
`--remote`. They only change the SSH path; they do not configure routing or
Internet sharing.

## Start order on the robot

1. physical emergency stop checked;
2. robot mechanically secured for the first run;
3. CAN and sensor interfaces identified;
4. network route checked;
5. no old Docker stack running;
6. compile completed;
7. default configs reviewed;
8. live stack started;
9. health, TF, topic and command checks run;
10. low-speed motion test performed in a clear area.

Commands:

```bash
docker ps
./scripts/compile
cd demos/live_robot
docker compose up
```

Use a second terminal for checks. Use `Ctrl-C` and `docker compose down` for a
normal stop.

## Repository ownership

| Change | Repository |
| --- | --- |
| Docker, Compose, scripts, manifests, workspace docs | `mtt_workspace` |
| MTT driver, control, localization, description, perception | `src/mtt_core` |
| sensor launches, recording, Zenoh router, integration | `src/external/norlab_robot` |
| teach-and-repeat algorithm | `src/external/wiln` |
| ICP mapper implementation | `src/external/norlab_icp_mapper_ros` |
| third-party library fix | its own `src/external/<repo>` |

Do not leave a fix only as a dirty nested repository. The parent cannot store
the nested diff; it stores only the nested commit ID.

## Correct multi-repository commit order

For every modified nested repository:

```bash
git -C src/<repo> status
git -C src/<repo> add <files>
git -C src/<repo> commit -m "short message"
git -C src/<repo> push
```

Then update and commit the parent pointers:

```bash
git status
git add src/<repo> .gitmodules dependencies/robot.repos
git commit -m "freeze workspace"
git push
```

Never push the parent pointer before the nested commit exists on a remote that
the next operator can access.

## Freeze checklist

```bash
./scripts/compile
./scripts/status --doctor --summary
git diff --check
git submodule status --recursive
```

Required result:

- build succeeds;
- doctor has no failures;
- no nested repository is dirty;
- no submodule line starts with `-`, `+` or `U`;
- all nested commits are pushed;
- parent branch is pushed;
- a clean clone can run `./scripts/setup`.

Tagging is optional, but a deployment tag is clearer than a branch name:

```bash
git tag -a mtt-handover-YYYYMMDD -m "MTT handover"
git push origin mtt-handover-YYYYMMDD
```

## Qualification status at handoff

The Docker build completes 66 packages. Compose files and shell syntax pass.
The current WILN, GPS, avoidance, driver C++ and command-timeout functional
tests pass. The complete historical test tree is not green: one perception EKF
convergence test, several upstream ROMEA path tests and legacy lint suites still
fail. Do not describe this snapshot as fully qualified until those failures and
the physical acceptance tests are closed.

Exact findings: [robotics safety audit](./robotics_safety_audit_2026-08-17.md).

## Backup and data policy

Git contains source and documentation, not field data.

- bags: `data/`;
- generated reports and maps: `artifacts/` or `results/`;
- disposable experiments: `.scratch_test/`;
- manual backup copies: `*.bak.*`;
- paper-specific work: keep in the research location or an ignored output
  directory.

Back up field data separately. A Git push does not back up ignored files.

## If something fails in the field

Do not change several subsystems at once.

1. stop motion and secure the robot;
2. save logs and the exact `.env`/runtime config;
3. run `./scripts/audit_network_field.sh` if the problem is communication;
4. run the live `field_ready`, `icp_check`, `audit_tf` and `audit_topics` checks;
5. inspect one command path at a time;
6. reproduce with a bag before changing control or mapping logic.

Network details: [networking.md](./networking.md).
Zenoh details: [zenoh.md](./zenoh.md).
Runtime details: [live robot README](../../demos/live_robot/README.md).
