# Internship handover

The workspace coordinates several repositories. A handover consists of their
exact source revisions, reproducible checks, documented calibration, and an
accessible copy of required datasets and build images. A clean parent commit
alone does not capture local changes inside submodules.

## Navigation

| Responsibility | Location | Starting document |
|---|---|---|
| Operate the robot | `demos/live_robot`, shared `demos/common/config` | [live instructions](../../demos/live_robot/README.md) |
| Acquire experiments | `demos/data_collection` | [collection instructions](../../demos/data_collection/README.md) |
| Runtime implementation | `src/mtt_core`, selected external integration repos | [core reference](mtt_core.md) |
| Research and saved data | historical tools in `scripts/`, new tools in `scripts/research/` | [research guide](../../scripts/research/README.md) |
| Installation and maintenance | `scripts/setup`, `verify`, `status`, `pull` | [developer guide](development.md) |
| Calibration | source Xacro and recorded session metadata | [calibration reference](calibration.md) |
| Current findings | audit and generated test logs | [2026-09-26 audit](robotics_safety_audit_2026-09-26.md) |

## Checks before a release

From the workspace root:

```bash
./scripts/verify
./scripts/test_offline
./scripts/status --doctor --summary
git diff --check
git submodule status --recursive
```

`verify` checks the maintained parent source surface, local imports/helpers,
Markdown links, shell/Python syntax, all Compose profiles, tooling regressions
and repository structure. It does not execute research pipelines or prove
absence of runtime bugs. CI runs a subset without private submodule access.

`test_offline` creates a new build directory inside `artifacts/offline-tests`.
Its container has no network, no host devices and a read-only source mount.
It builds messages, control, driver, perception and GPS parsing, then runs
registered behavioral tests plus pure avoidance tests. It uses an existing
Docker image; it does not validate a fresh Docker image build or all external
packages. Lint and physical acceptance remain separate requirements.

Record the resulting logs with the exact revisions. Never replace a failed
test result with a historical successful build or an empty test directory.

## Commit and publication order

Review independently:

1. `src/mtt_core`: control validation/tests, build configuration, calibration
   and any other runtime changes. Keep unverified physical calibration separate
   from validated software fixes.
2. `src/external/norlab_robot`: recorder/integration configuration.
3. Parent: tooling, documentation, research dependencies and experiment profile.

Commit and publish the reviewed child changes first; then update and commit the
parent pointers. Check that the receiving account can read every pinned commit.
After publication, perform a new recursive clone using that account and run
the checks above. Finally run `./scripts/verify --release` on the frozen tree.

`scripts/pull` now follows the parent's submodule pins. Intentional dependency
upgrades are separate reviewed changes. Do not run a broad add over the local
archive or nested `norlab_ws`; select source paths explicitly.

Local logs, editor state and machine-specific workspace configuration are excluded
from the source set. Code comments should refer to maintained technical
documentation and recorded measurements. Preserve scientific provenance and
existing Git history when preparing a professional release.

## Source and data preservation

```bash
python3 scripts/handover_snapshot.py --output artifacts/handover/source-snapshot
```

This refuses an existing destination and writes `sources.tar.gz`, its SHA-256,
a file/repository manifest and a patch per initialized repository. It includes
current tracked source plus non-ignored maintained files and local research
helpers in `scripts`/`research` (also root Python helpers), preserving current
edits. Ignored documentation and local working notes are not included.
It does not contain Git objects/history, `.env`, bags, Docker images or
generated results. Missing submodules are recorded in the manifest. Preserve
the original clone too if Git history will be needed after access ends.
This is a local preservation tool, not a secret scanner: review the manifest
before sharing it, especially when adding local source or configuration.

`data` currently points outside the repository; copying the workspace does not
copy its target. Give the receiving owner a separate dataset inventory and
accessible storage location. Save the tested Docker image with `docker image
save` if rebuilding later must not depend on moving upstream packages.

## Acceptance still owned by the lab

Record the repository/data owner, operator and acceptance date. For the frozen
revision, verify on secured hardware: manual deadman, physical/software stop,
AUTO entry/exit, source/sensor loss, network reconnect, arbitration of simultaneous
sources, articulation return and COM stopping. Resolve the displacement-direction
question in the calibration reference before calling that calibration validated.
