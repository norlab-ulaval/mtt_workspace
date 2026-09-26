# Robotics Safety Code Audit Report

## 0. Scope and limits

- Repository: `mtt_workspace`, with focused review of MTT-owned control/driver
  code and the checked-out dependency manifests.
- Branch/commit at entry: `main`, `6470546`; `mtt_core` at `e0da59c`.
- Date: 2026-09-26.
- First pass: read-only inspection, Graphify query, Git status/diffs/submodule
  status, Python AST analysis, relative Markdown links, `./scripts/verify`,
  Docker image inventory. No claim of exhaustive verification of vendor code.
- Commands intentionally not run: live Compose stacks, setup/deployment/sync,
  network configuration, CAN/EtherCAT tools, experiment conductors, bag replay.
- Hardware not contacted: no robot, actuator, sensor or live ROS graph.
- Main uncertainty: physical acceptance and a fresh image build remain separate
  release requirements. The August audit is a
  historical result, not evidence that today's working tree is qualified.

## 1. Executive summary

The completed offline checks cover 126 Python files, eight Compose files,
40 workspace regression tests and the 34 recursive submodule pins. Git object
connectivity checks passed in all 35 repositories, including the parent. No
unresolved merge conflicts or broken relative Markdown file links were found
on the maintained source surface.

The **working tree at entry was not release-ready**: five tracked paths were modified (including two dirty submodules), and research scripts/configuration
were untracked or ignored. Successful historical builds cannot certify these
changes. Several concrete reproducibility and command-validation defects were
identified below. Corrections and fresh validation are recorded after the
first-pass findings, without erasing pre-existing work.

### Corrections and regressions

- **Commands (H01/H02):** future, expired, zero and negative timestamps are
  rejected; future commands cannot become active merely by waiting. Any
  nonfinite Twist component produces zero output, and unsafe startup parameters
  fail explicitly. The actual arbiter executable passes 32 isolated ROS tests.
- **Git and verification (H03/H04):** development checks allow ordinary edits;
  release checks still require clean repositories. Pull refuses dirty trees
  before updating and restores the exact child pins, including detached HEADs.
- **Research completeness (H05):** required ignored helpers are included in the
  source set. The missing `lib/mtt_motion_research.py`, imported by six scripts,
  was recovered from the local deleted-file archive. Six command-line entrypoints
  and a synthetic CSV-to-dataset run pass without ROS or experimental data.
- **Research correctness (H14):** regression tests reproduce and fix zero IMU
  rate becoming NaN, the legacy ICP heading alias failing on later rows, and
  speed sign reversal when heading crosses pi. Unknown/synthetic tachometer
  provenance no longer becomes “real”; nonpositive time intervals are rejected
  by the research quality gate. Existing datasets/results were not overwritten.
- **Calibration output (H13):** candidates retain all sensor properties and
  template yaw; unestimated components keep template values. Output overwrite
  and nonfinite candidates are refused. Physical extrinsic values are unchanged.
- **Build (H15):** fixed a fresh-build failure caused by bare Eigen definitions
  from the installed OR-Tools vendor configuration reaching compiler arguments
  through PCL's legacy CMake integration.
- **Test contract (H16):** retained the 1 cm convergence target with noiseless
  observations and an offset initial state; added a distinct seeded noisy-case
  check against the four-dimensional 99% covariance bound. No estimator runtime
  tuning changed. These tests still exercise reference copies of algorithms.
- **Handover (H07/H09/H11):** documented robot/research entrypoints, normalized
  steering semantics, calibration limits, commit order and source preservation.
  Docker exclusions cover local data/archives/environments; irrelevant local
  working-note references were replaced with maintained technical sources.

### Validation evidence

The six-package build (`mtt_msgs`, `mtt_interfaces`, `mtt_control`, `mtt_driver`,
`mtt_perception`, `mtt_gps_driver`) used a new build/install tree in
`artifacts/offline-tests/run.65pJYW` and the existing image recorded in
`image-id.txt`. After fixes, changed packages were rebuilt in that tree.
Containers had `--network none`, no host devices and read-only source mounts.

- 82 distinct registered behavioral cases pass across control, driver,
  perception and GPS. Colcon reports 93 because it also counts the 11 CTest
  wrappers; do not describe this as 93 independent cases.
- 11 pure Python avoidance tests pass. Final command exit codes are all zero
  in `exit-codes.txt`; see `tests-verified.log`, `test-results-verified.log` and
  `avoidance-verified.log`. Earlier failing logs remain available as evidence.
- The initial arbiter implementation failed 14 of the first 28 regression cases;
  the final executable passes all 32, including subsequently added negative-time
  and future-command reactivation cases.
- Both v1/v2 Xacro files expand into structurally valid URDF trees: 52 links,
  51 joints, one root, finite origins and no duplicate parents/cycles.
- A temporary copy containing only publishable parent sources, without local
  ignored modules or initialized submodules, passes source checks, tool tests
  and all Compose parses. GitHub Actions itself has not run in this session.

The final runner records individual command exit codes in `exit-codes.txt`.
A minimal reproduction showed that the image's login-shell logout hook returned
an error even for `set -e; exit 0`. The runner now uses a non-login shell and
sources ROS explicitly, so user login/logout hooks cannot override test results.

### Remaining release conditions

The original calibration displacement note conflicts with the encoded direction:
v2 is **+0.230829 m forward** in the current frame. The revised comment records
this discrepancy without changing the translation.
Resolve this against the mounting measurement/static bag before calling v2
validated. See [calibration details](calibration.md).

Changes are grouped into reviewable commits: command validation, perception
build/test corrections, calibration mounting history, recorder configuration,
workspace verification and research tools. Child repository changes are published
before the parent pins. See [handover procedure](handover.md) for the final
clean-tree and remote checks. No execution logs are part of the source release.

This is neither a fresh Docker image rebuild nor an exhaustive vendor/lint test
run. PCL CMake-policy and ignored `nodiscard` warnings remain visible. Integrated
clock loss, reconnection, actuator/COM stopping and measured calibration still
require secured hardware acceptance. Offline success is not physical qualification.
The new finite-input gate protects the arbiter path. Direct publishers into
the CAN driver's command topic bypass that gate; driver-level invalid-input
acceptance and the separate steering/COM paths still need dedicated coverage.

### Final review of acquisition and export tools

The frozen Session-A profile retains its recorded SHA-256 and all 60 unique
attempt IDs. Tests check interpolation, cosine boundaries, malformed profiles,
telemetry freshness/provenance, clock discontinuities and transition-event
flags. The confirmatory monitor is opt-in and advisory; it does not command
motion or establish scientific qualification. Missing timing samples and
non-increasing timestamps now fail post-session qualification.

Root CSV exporters require explicit bag paths, refuse output collisions and
keep normalized command values separate from angular units. Help/import and
bag-selection tests pass; full exports still require recorded bags and the ROS
image. These checks do not certify every research pipeline or scientific result.

## 2. Risk table

| ID | Severity | Confidence | Area | Finding | Runtime effect | Evidence | Next action |
|---|---:|---|---|---|---|---|---|
| H01 | S1 | confirmed | commands | Arbiter freshness accepts negative age | Future-dated commands can survive the source timeout while being restamped for the driver | `mtt_control/src/nodes/mtt_cmd_arbiter_node.cpp`, `cmd_is_fresh`, `on_timer` | Test future/expired/zero timestamps and reject negative age |
| H02 | S1 | confirmed | commands | Arbiter accepts nonfinite Twist fields; initial AUTO speed limit lacks the live parameter validator | Invalid data can reach the final command/driver path | arbiter `on_manual_cmd`, `on_auto_cmd`, constructor; driver `on_cmd_vel` uses `std::clamp` without a finite check | Test invalid commands and startup parameters; add a final arbiter gate |
| H03 | S2 | confirmed | verification | Ordinary `verify` fails on a dirty worktree although `--release` claims to add that requirement | Developers cannot distinguish a failed check from normal edits | `scripts/verify` calls `status --doctor`; status final exit policy | Separate structure checks from release cleanliness, test both |
| H04 | S2 | confirmed | Git | `pull` follows child branches and assumes `mtt_core` has a branch | Fresh submodule clones have detached HEADs; updates can leave parent pins | `scripts/pull` | Update submodules to parent pins; refuse dirty trees before pulling |
| H05 | S2 | confirmed | research | Tracked code imports ignored modules | A clean clone loses reference conversion and optional geofence support | `build_gt_reference_csv.py:20`, conductor/monitor imports of `zone_map`; `.gitignore` | Include reusable dependency sources and test clone completeness |
| H06 | S2 | confirmed | handover | Required changes and confirmatory tools are local only | Parent SHA does not capture the tested files | entry Git status; dirty calibration and recorder config | Review and publish child changes before parent release; archive data separately |
| H07 | S2 | confirmed | Docker | Docker context omits exclusions for artifacts, ZIPs and nested virtual environments | `COPY . .` can bake local datasets, scratch code and a 493 MB ZIP into the image | `.dockerignore`, `docker/Dockerfile`; root `mapping_results_27aout.zip` | Exclude generated/local material explicitly |
| H08 | S2 | confirmed | dependencies | Several build inputs are moving branches/downloads and broad pip constraints | Same source commit can produce a different image later | Dockerfiles: SOEM/libnabo/libpointmatcher clones, apt/pip, vendor downloads | Record image digest and dependency versions; pin only tested versions |
| H09 | S2 | confirmed | semantics | Generic angular rad/s description conflicts with current driver mode | Generic Twist publishers can request the wrong steering value | `demos/common/config/mtt_driver.yaml`: `normalized_steer`; driver `command_angular_to_normalized_steer` | Document normalized steering versus yaw-rate mode explicitly |
| H10 | S2 | needs human validation | safety | No complete acceptance evidence for stop, stale inputs and reconnect across all paths | Unit tests alone cannot establish physical stopping behavior | control, driver, articulation-servo and COM paths; August audit | Isolated software tests, followed by secured hardware acceptance |
| H11 | S3 | confirmed | maintainability | Long scripts mix extraction, numerical processing, plotting and orchestration | Changes are hard to review and reproduce independently | `validate_motion_model.py` 2056 lines; `run_ros_healthcheck` 494 lines; `qualify_gt_v2.main` 419 lines | Add entrypoint catalog and workflow boundaries; extract pure logic incrementally |
| H12 | S2 | confirmed | calibration | Backward displacement note conflicts with positive X translation in an unrotated forward frame | Mount interpretation can be wrong despite valid XML | `calib_v2.xacro`, `reference_point_joint`; v2 minus v1 X = +0.230829 m | Physical measurement required; numbers preserved |
| H13 | S2 | fixed and tested | calibration tool | Partial generated Xacro omitted sensor properties and reset yaw | Generated file could fail expansion or change unrelated mounts | `scripts/calib_hesai_scan.py`, `tests/test_calibration_xacro.py` | Complete candidate from template; no overwrite |
| H14 | S2 | fixed and tested | research | Wrapped heading, zero values, legacy schema and quality metadata were mishandled | Wrong speeds, lost measurements or unsupported sample acceptance | `scripts/lib/mtt_motion_research.py`, `tests/test_research_entrypoints.py` | Five reproduced defects fixed; regenerate affected analyses with recorded version |
| H15 | S2 | fixed and tested | fresh build | Bare `EIGEN_MPL2_ONLY` reaches compiler as a filename | Perception cannot compile with current vendor Eigen discovery | `mtt_perception/CMakeLists.txt`, initial/fixed build logs | Normalize definitions with CMake; fresh tree compiled |
| H16 | S3 | fixed test contract | perception test | Noisy posterior was required to stay within 1 cm regardless of covariance | Persistent red test obscures regressions; not proof of a runtime defect | `mtt_trailer_estimator_test.cpp`, original Y error 0.033531 m | Separate deterministic convergence and seeded covariance-consistency checks |

## 3. Command authority map

```text
joy -> operator input -> manual filter -> cmd_vel/manual -----------+
WILN / GPS / experiment -> autonomy mux -> controller/cmd_vel ------+-> arbiter
monitor constant_speed -------------------------------------------+    |
mode / teleop estop / deadman ------------------------------------------+-> cmd_vel
                                                                          |
                                                       CAN driver -> CAN 0x100
articulation setpoint -> articulation servo -> driver steering override ---+
COM tools -> separate motor/EtherCAT path (not proven covered by cmd_vel stop)
```

Direct actuator diagnostics and simulation/replay publishers must not share a
live graph with this chain. The `constant_speed` profile is command-capable.

## 4. Mode and state machine analysis

| Arbiter state | Gate | Output |
|---|---|---|
| STOP/startup | default selected mode | zero |
| transition | mode change hold, default 0.15 s | zero |
| MANUAL | fresh manual command and deadman unless disabled | manual command |
| AUTO | AUTO enabled and fresh autonomous command | longitudinally limited AUTO command |
| ESTOP | teleop estop active, before all mode checks | zero |

Separate states exist in the autonomy mux (idle/WILN/GPS/experiment) and repeat
supervisor. Multiple final publishers, simultaneous live projects, and direct
actuator tests during replay are forbidden combinations. Stop release and
independent Boolean topic freshness need integrated tests; do not infer them
from one callback.

## 5. Units, frames, signs, and timing

- Driver linear command is m/s. With committed `normalized_steer`, angular.z is
  a dimensionless steering request in [-1, 1], not a generic yaw rate.
- Articulation setpoints are radians; confirm each adapter before connecting a
  new planner. Keep body/track-contact-center distinctions in research data.
- Arbiter source timeouts use ROS header time; driver reception timeout uses
  steady time. Restamping is why H01 defeats the downstream freshness defense.
- The pre-existing `calib_v2.xacro` mounting correction is preserved. This audit
  does not retune extrinsics or certify the physical mounting.

## 6. ROS 2 communication and QoS

The arbiter consumes manual/AUTO `TwistStamped`, mode `String`, and enabled,
deadman and estop `Bool`, with depth 20. Selected-source and AUTO limit outputs
are transient-local depth 1. The autonomy mux pairs Twist and articulation
inputs with timeouts. The CAN driver consumes final Twist with depth 10.

The observation above is a source review, not a full live QoS compatibility
test. The recorder topic list now includes confirmatory events; its QoS has
not been changed. Acceptance must use the pinned configuration.

## 7. Safety mechanisms

Observed: default STOP, transition hold, manual deadman, estop priority,
source timeouts, steady-clock driver timeout, telemetry watchdog, obstacle
monitor stale-cloud stop and repeat readiness checks. Physical e-stop and
firmware behavior cannot be established from this workspace.

Regression priorities for H01/H02: zero/future/expired stamps, NaN/Inf,
startup STOP, manual deadman release, AUTO disable, estop precedence, speed
limit validation. H10 additionally requires process loss, clock pause/jump,
network loss/reconnect and independent articulation/COM stop checks.

## 8. Hardware, network, Docker, and driver risks

Live Compose services expose host networking and devices intentionally.
`docker/common.yaml` also contains machine-specific `/data` and LaCie mounts.
Tests should use an isolated container with no network and no host devices,
not inherit the live Compose base. Exclude local archives and scratch work
from Docker context. Preserve tested image IDs alongside the release.

## 9. Code complexity and maintainability

Strengths: separated command nodes, named units in many parameters, explicit
timeouts, pure driver/control logic with tests, central demo configuration,
submodule pins, provenance helpers, and readable workflow documentation.

Weaknesses: research and runtime entrypoints coexist; ignored dependencies
undermine the structure; long functions combine unrelated responsibilities;
several comments describe old field sessions rather than durable contracts.
Prefer small extractions protected by behavior tests. Mass relocation or
reformatting would make this handover harder to review.

## 10. Tests missing before safe refactor

1. Added: arbiter invalid input, timestamp, parameter and priority regressions.
2. Added: Git workflow tests using local repositories (detached HEAD, dirty child).
3. Added: verification tests with deliberately broken Python/conflict markers.
4. Added: research import/entrypoint checks and synthetic numerical data.
5. Secured hardware matrix from H10. No claim of physical qualification until
   the owner records results for the exact frozen commits.

## 11. Behavior-preserving refactor plan

### PR 1 — Tests and observability only

- Files: workspace validation/tests and audit/handover docs.
- Reason: make failures visible and repeatable; runtime behavior change: none.
- Tests: synthetic fixtures and no-hardware checks. Rollback: revert this patch.

### PR 2 — Constants and units

- Files: command documentation and parameter descriptions.
- Reason: prevent normalized-steering/yaw-rate confusion; behavior change: none.
- Tests: compare docs with active config. Rollback: documentation-only revert.

### PR 3 — Command arbitration clarity

- Files: arbiter implementation and regression tests.
- Reason: reject invalid age/data/startup limits; behavior change: explicit
  rejection of invalid inputs, valid command selection unchanged.
- Tests: H01/H02 matrix. Rollback: revert the isolated fix; do not deploy a
  rollback without recording that it restores the known defects.

### PR 4 — Safety gate isolation

- Files: future driver/servo/COM safety tests and pure gate logic.
- Reason: cover every actuation path; behavior change: none intended.
- Tests: stop precedence and reconnect tests before extraction.
- Rollback: revert extraction separately. Deferred pending those tests.

### PR 5 — Controller/math extraction

- Files: future research numerical helpers.
- Reason: separate input/output from model mathematics; behavior change: none.
- Tests: synthetic trajectories and saved-data regression; rollback: restore
  previous entrypoint. Do not tune scientific models during repository cleanup.

### PR 6 — Launch/config validation

- Files: verification, dependency ownership and Docker exclusions.
- Reason: make clean clones usable; runtime behavior change: none intended;
  invalid workspace/release states should fail explicitly.
- Tests: Compose parsing, script syntax, Git fixtures and source inventory.
- Rollback: revert individual tooling/configuration commits.

## 12. Do-not-touch-yet list

Existing user changes to calibration, recorder QoS and confirmatory protocol;
CAN packet encoding, physical stop semantics, EtherCAT enable/reconnect;
mapping thresholds, scientific ground-truth assumptions and controller tuning.
These need their own measured acceptance evidence, not cosmetic cleanup.

## 13. Questions for the human robotics owner

Before deployment/final publication, record who owns the repositories and
data after the internship, and who signs the secured hardware acceptance.
Confirm stop coverage of articulation and COM with the actual firmware.
Confirm whether the cage moved forward or backward between the two measured
mount states; successful Xacro expansion cannot answer this physical question.
These decisions do not block offline checks or the targeted tooling fixes.
