# Research and post-processing

Use this guide for saved-data analysis. Robot execution belongs to `demos/` and
ROS packages under `src/mtt_core`; installation/verification belongs to the
workspace administration tools listed in [scripts/README.md](../README.md).

Existing scripts keep their paths for compatibility with recorded commands and
imports. Put new experiment-specific entrypoints in this directory; put shared
pure helpers in `scripts/lib/`. Generated outputs belong under `artifacts/` or
the selected session directory. Do not move a script without checking imports,
subprocess callers, Compose and recorded dataset provenance together.

## Entry points and ownership

| Task | Entry points in `scripts/` | Inputs / environment |
|---|---|---|
| Inspect saved bag timing/topics | `audit_bag_timing.py`, `audit_bag_topics.py` | recorded bag; ROS container for deserialization |
| Build offline mapping reference | `build_gt_pipeline.py`, `offline_reference.py` | explicitly approved ICP CSV, bag, calibration and compiled solver |
| Export a reference/dataset | `build_gt_reference_csv.py`, `build_session_dataset.py`, `build_msa_canonical_dataset.py` | CSVs with documented frame/time schema; NumPy, pandas, SciPy, PyYAML |
| Export a bag preview | `export_bag_preview.py` | bag; ROS Python, OpenCV, ffmpeg |
| Export colored PIN-SLAM clouds | `export_pinslam_colored_clouds.py` | MCAP with static TF and camera metadata; NumPy, OpenCV, Open3D, rosbags |
| Qualify confirmatory acquisition | `qualify_confirmatory_session.py` | recorded bag and explicit `--topic-contract`; ROS environment |
| Build quality-labelled research table | `build_motion_research_dataset.py` | postprocess CSV; `lib/mtt_motion_research.py`, PyYAML |
| Evaluate a model suite | `evaluate_motion_model_suite.py`, `evaluate_command_model_progression.py`, `kfold_motion_model_tuning.py` | research table and model configuration |
| Plot/diagnose | `plot_motion_research_report.py`, `diagnose_motion_model_failure.py`, `analyze_ice_session.py` | CSV results; plotting dependencies as imported by the script |
| Historical baseline implementations | `mtt_motion_model/` | study-specific geometry/sign conventions; see module docstrings |
| Live acquisition | `mtt_experiment_conductor.py` | **commands the robot**; follow the data-collection README |

Run `python3 scripts/<entrypoint>.py --help` for exact flags. Scripts that import
ROS at module level require the container even for help. Import/help checks do
not establish correctness of scientific outputs.

## Reproducibility record

For every result retained for publication, store:

1. Input file hashes and session identifier; keep original bags immutable.
2. Parent and relevant submodule SHAs, plus any working-tree patch.
3. Exact command, calibration file/hash, reference source and qualification.
4. Python/dependency versions or tested Docker image ID and an archived image.
5. Model family, geometry, articulation source/sign, units and reference frame.
6. Train/test session split, quality exclusions and output hashes.

`gt_provenance.py` and `gt_catalog.py` provide provenance and frozen-dataset
protection. Keep those checks active. The [older pipeline note](../../docs/motion_model_research_pipeline.md)
describes one historical baseline; it does not define every M0–M5 family.

The restored `lib/mtt_motion_research.py` has historical nominal geometry
0.9/1.5 m and positive curvature for positive articulation. Other models use
different measured geometry or opposite articulation conventions. Record the
implementation, not only a label such as “M1”. See
[calibration and geometry](../../docs/reference/calibration.md).

## Preserving local work

The workspace has additional ignored analysis scripts and local datasets.
`git status` alone does not inventory ignored work. Create a source snapshot
with `handover_snapshot.py` and inspect its manifest before access ends.
Sources included in that archive are not automatically qualified or published.

The snapshot excludes bag data, generated maps, images, nested workspaces,
virtual environments and Git history. Back up those separately when needed.
Promote a local tool into the maintained source set only with its dependencies,
documented inputs and an offline validation result.
