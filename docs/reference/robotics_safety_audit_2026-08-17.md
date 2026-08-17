# Robotics Safety Code Audit Report

## 0. Scope and limits

- Repository: `mtt_workspace` and all checked-out repositories under `src/`
- Branch/commit: parent `main` from `0dcee3d` through the handoff freeze; exact
  nested commits are pinned by the handoff commit
- Date: 2026-08-17
- Commands run: Graphify queries, Git status/diff/submodule inspection, static
  source/config searches, shell syntax checks, Compose config validation and a
  full Docker `colcon build`
- Commands intentionally not run: live launch files, CAN setup, EtherCAT,
  joystick, actuator scripts, ROS topic publication and field network changes
- Hardware not contacted: no robot, CAN bus, EtherCAT device, sensor or live ROS
  graph was intentionally contacted
- Main uncertainty: current control, mapping and hardware changes have not been
  validated on the physical robot as one integrated release

## 1. Executive summary

The workspace builds: 66 packages completed in Docker, including KISS-ICP from
source. All checked Compose
files parse and the shell scripts pass `bash -n`.

The handoff pass commits and pushes every required nested change, aligns the
repository manifests and pins those commits in the parent. Live defaults remain
hardware-capable and the laptop monitor has an opt-in fixed-speed publisher, so
offline and live paths remain deliberately separate.

## 2. Risk table

| ID | Severity | Confidence | Area | Finding | Runtime effect | Evidence | Next action |
| --- | ---: | ---: | --- | --- | --- | --- | --- |
| R1 | S1 | confirmed | command | monitor `constant_speed` publishes a nonzero autonomous command | starting the command profile can move a connected robot | `demos/monitor/compose.yaml`, service `constant_speed`, default `0.20` m/s | keep opt-in, document as command-capable and test only secured |
| R2 | S1 | needs human validation | safety | no repository-level integration test proves physical e-stop priority over every current manual/autonomous path | a regression could leave one command path active | unit safety tests exist under `mtt_driver/test`, but no complete hardware/launch arbitration test was found | add isolated arbitration launch test, then physical acceptance test |
| R3 | S2 | resolved in handoff | release | parent and nested repositories were dirty | clean clone did not reproduce the tested build | nested commits were pushed before the parent pointer | keep `./scripts/verify --release` green |
| R4 | S2 | resolved in handoff | dependencies | `.gitmodules` and `robot.repos` disagreed for ICP and Badger | setup content depended on the import method | manifests are aligned and doctor checks them | update both manifests together |
| R5 | S2 | resolved in handoff | source | new `mtt_gps_teach_repeat` package was untracked | package would disappear from a clone | package is committed and pushed in `mtt_core` | keep package tests in the normal build |
| R6 | S2 | confirmed | network | Zenoh and Foxglove field endpoints use plain TCP without workspace auth/TLS | trusted-LAN service could be exposed accidentally | Zenoh client/router templates and Compose ports 7447/8765/8766 | keep on trusted networks or add a secured tunnel/deployment |
| R7 | S2 | confirmed | Docker | live services use host networking, devices and privileged access | container can reach host hardware and network directly | `docker/common.yaml`, live/data Compose files | keep live stacks robot-only and offline demos separate |
| R8 | S2 | confirmed | network | live `runtime.env` defaults to Doodle mode and fixed field addresses | wrong physical network can make monitoring unavailable or bind to the wrong interface | `demos/live_robot/config/runtime.env` | verify interface/IP before every deployment |
| R9 | S2 | resolved in handoff | repository | Badger `.gitignore` contained unresolved conflict markers | doctor failed and future ignores were ambiguous | markers were removed and Python caches are ignored | keep doctor conflict scan enabled |
| R10 | S3 | confirmed | maintenance | operational and research scripts share one flat directory | new operator can run the wrong tool or commit generated research output | `scripts/` inventory | maintain a risk index and put new research tools under `scripts/research/` |
| R11 | S2 | confirmed | tests | the full offline test run is not green | regressions can be hidden behind a successful build | one MTT perception convergence test and ROMEA path tests fail; MTT lint suites also fail | keep failures visible and repair before claiming a fully qualified release |
| R12 | S2 | resolved in handoff | Python | pip installed incompatible NumPy and setuptools versions | pytest/plugin or colcon startup could fail | image now uses NumPy 1.26.4 and setuptools 79.0.1; isolated Python tests run | keep compatibility constraints in Dockerfile |

Severity uses S0 immediate physical risk, S1 realistic unsafe behavior, S2
reliability that can become unsafe, S3 maintainability and S4 style.

## 3. Command authority map

```text
/joy
  -> mtt_operator_input_node
  -> cmd_vel/manual_raw
  -> mtt_manual_cmd_filter_node
  -> cmd_vel/manual -----------------------+
                                             +-> mtt_cmd_arbiter_node -> cmd_vel -> mtt_can_node -> CAN 0x100
WILN / route / path follower                 |
  -> controller/cmd_vel --------------------+

mode manager, deadman, estop, source freshness and driver command timeout
  -> gate or neutralize the final command path
```

The monitor `constant_speed` service can publish directly to
`controller/cmd_vel`, so it enters the autonomous side of the same arbiter.

## 4. Mode and state machine analysis

Documented operator modes are MANUAL, AUTO, STOP/ESTOP, startup/fault and idle.
Joystick activity with deadman held is intended to force MANUAL. WILN replay
requires AUTO and should be cancelled on manual override or stale localization.

Forbidden combinations:

- two live Compose projects publishing commands;
- two final `cmd_vel` publishers;
- autonomous replay while a direct command test runs;
- real COM motor profile during a dry-run assumption;
- duplicate TF owners from replay and live stacks.

Integrated transition testing is still required for the current WIP.

## 5. Units, frames, signs, and timing

- documented command units: linear m/s, angular rad/s;
- primary body convention: x forward, y left, z up;
- primary chain: `map -> odom -> base_footprint -> base_link -> sensors`;
- driver command and telemetry timeouts default to 0.5 s in launch files;
- current mapper deskew is disabled in live runtime because wheel/TF deskew was
  observed to amplify slip/articulation error;
- TF ownership must remain unique during replay; recorded TF is excluded by
  default in the replay stack.

Mapping thresholds and frame corrections are safety-relevant WIP and were not
changed by this documentation pass.

## 6. ROS 2 communication and QoS

Live cross-machine communication uses `rmw_zenoh_cpp`, TCP router port 7447 and
`LIVE_ROBOT_DOMAIN_ID=2`. Monitor clients disable multicast and gossip and use
one explicit endpoint. Recording uses a curated topic list and QoS overrides in
`src/external/norlab_robot/config/rosbag_record/`.

Publisher/subscriber QoS compatibility must still be verified per missing
topic. Zenoh transport does not repair incompatible ROS QoS.

## 7. Safety mechanisms

Observed mechanisms:

- physical emergency stop outside the software stack;
- teleop e-stop safety lock in the driver wrapper;
- joystick deadman and activity thresholds;
- manual/autonomous mode arbitration;
- final command freshness timeout;
- CAN telemetry timeout;
- localization/ICP readiness gates for replay;
- obstacle monitoring and replay supervisor paths.

Startup, shutdown, reconnect and complete e-stop priority still require one
integrated acceptance test on the frozen commits.

## 8. Hardware, network, Docker, and driver risks

Live containers use host networking and may mount `/dev`, USB, input devices and
GPU resources. CAN uses `can0`; Badger/PCAN feedback uses `can32`; COM uses an
EtherCAT interface currently configured as `enp6s0`. Sensor networks and Doodle
must not become default routes.

Zenoh/Foxglove field traffic can exceed measured Doodle capacity when raw clouds
or images are open. Use the reduced field allowlist and voxelized cloud.

## 9. Code complexity and maintainability

The active WIP spans command arbitration, localization, WILN, mapper configs,
recording QoS and Compose orchestration. These areas should not be combined into
one unreviewed behavioral refactor. The large flat script directory also hides
the boundary between deployment and research.

## 10. Tests missing before safe refactor

Priority order:

1. command arbitration with simultaneous manual, autonomous and fixed-speed
   publishers;
2. emergency stop priority and release behavior;
3. stale command, joystick and localization tests;
4. startup/shutdown neutral command test;
5. Zenoh disconnect/reconnect without stale command replay;
6. Compose launch smoke tests with hardware disabled;
7. route replay cancellation on manual override;
8. QoS compatibility check for the curated recorder list.

## 11. Behavior-preserving refactor plan

### PR 1 — Tests and observability only

- files touched: control/driver tests and diagnostics
- reason: freeze current authority and timeout behavior
- behavior change: expected none
- tests: arbitration, e-stop, stale source, startup/shutdown
- rollback: revert test-only commit

### PR 2 — Constants and units

- files touched: configs and named constants only
- reason: remove ambiguous units and duplicated defaults
- behavior change: expected none
- tests: parameter equivalence and existing control tests
- rollback: revert constants commit

### PR 3 — Command arbitration clarity

- files touched: `mtt_control`
- reason: one explicit final command authority
- behavior change: expected none
- tests: multi-source priority matrix
- rollback: restore prior arbiter commit

### PR 4 — Safety gate isolation

- files touched: supervisor/driver safety boundary
- reason: make stop and freshness authority auditable
- behavior change: expected none
- tests: e-stop and disconnect cases
- rollback: restore prior safety gate

### PR 5 — Controller/math extraction

- files touched: pure control and path functions
- reason: test signs, frames and limits without ROS
- behavior change: expected none
- tests: unit, regression and bag replay
- rollback: restore in-node implementation

### PR 6 — Launch/config validation

- files touched: Compose, launch and config checks
- reason: reject missing, duplicate or contradictory runtime settings
- behavior change: explicit startup failure on invalid configuration
- tests: no-hardware Compose/launch smoke suite
- rollback: disable validation gate only after documenting why

## 12. Do-not-touch-yet list

- mapper thresholds and acceptance/recovery behavior;
- command arbitration and mode semantics;
- CAN command encoding, security switch and firmware assumptions;
- EtherCAT auto-enable behavior;
- TF extrinsics and hidden sign corrections;
- WILN path deformation/follower behavior;
- localization factor graph and offline reference solver.

Change these only after the relevant tests exist and the current WIP is frozen.

## 13. Questions for the human robotics owner

- Is the light switch still acting as the firmware emergency stop in the final
  vehicle firmware?
- Is CAN 0x100 still the only intended external command authority?
- Must Doodle mode remain the committed default, or should deployment select it
  locally?
- Which account and remote branches are the supported handoff targets for the
  modified nested repositories?
- Is COM auto-enable acceptable for the final field procedure, or must the
  committed default be dry-run/manual-enable?
