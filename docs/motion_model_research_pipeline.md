# MTT Motion Model Research Pipeline

## Ground Truth Contract

Use `offline_icp_canonical` as the reference only when `canonical_quality.status: PASS`.

Required files per session:

- `offline_icp_canonical/map.vtk`: ICP map in `map`.
- `offline_icp_canonical/trajectory.vtk`: ICP trajectory points in `map`.
- `offline_icp_canonical/icp_odom_replay`: rebuilt `/mapping/icp_odom`.
- `postprocess_dataset/dataset.csv`: synchronized sensor/input/ground-truth table.
- `postprocess_dataset/motion_model_dataset.csv`: model-ready table.
- `postprocess_dataset/audit.yaml`: physical consistency checks.
- `research_report/*.png`: human visual inspection.

Do not train from sessions with missing ICP, low ICP coverage, synthetic tachometer when real tachometer is expected, non-monotonic time, or suspicious tachometer/ICP speed sign.

## Frames

Mapping ground truth is in the `map` frame:

```text
map -> odom -> base_footprint -> base_link -> hesai_lidar
```

Motion-model states use the planar robot body frame:

- `x, y, yaw`: `base_footprint` pose in `map`.
- `v`: forward speed along robot +X.
- `phi`: articulation angle, positive according to the recorded MTT convention.
- `wz`: yaw rate around +Z.

The LiDAR has a static 90 deg yaw relative to the robot. This is handled by TF and must not be manually re-applied in motion-model code.

## Current Baseline Model

The implemented kinematic baseline is:

```text
kappa = tan(phi) / L
wz = v * kappa * slip(v, phi)
dx = v * cos(yaw + 0.5*wz*dt) * dt
dy = v * sin(yaw + 0.5*wz*dt) * dt
dyaw = wz * dt
```

with first-order command response:

```text
v_eff[k+1] = v_eff[k] + alpha_v * (v_target - v_eff[k])
phi_eff[k+1] = phi_eff[k] + alpha_phi * (phi_cmd - phi_eff[k])
```

and midpoint Euler integration. This is a correct baseline for a planar articulated vehicle if `L`, sign conventions, and `phi` represent the effective articulation at the hitch.

Main limitations:

- Longitudinal slip is not explicitly estimated.
- Yaw slip is currently heuristic.
- Acceleration/braking dynamics are simplified.
- Track-soil interaction is not state dependent.
- Articulation actuator dynamics are first-order only.

## Model Ladder

M0: Identity / recorded odometry baseline

- Compare `/mtt_odometry` directly against canonical ICP.
- Purpose: detect sign, scale, yaw-rate clamp, and TF errors.

M1: Pure kinematic articulation model

- Inputs: real tachometer speed + measured articulation.
- Parameters: effective wheelbase `L`, sign conventions.
- Equation: `wz = v * tan(phi) / L`.

M2: Kinematic + slip gains

- Inputs: tachometer speed, measured articulation, IMU yaw rate optional.
- Parameters: `L`, yaw slip gain as function of `|v|`, `|phi|`.
- Purpose: explain systematic yaw mismatch.

M3: Command-to-state dynamic model

- Inputs: `/cmd_vel`, throttle/brake, steering command.
- States: `v_eff`, `phi_eff`.
- Parameters: command sign, speed gain, speed time constant, articulation time constant, deadband.
- Purpose: predict motion from commands, not from measured hardware.

M4: Hybrid learned residual model

- Inputs: command, tachometer, articulation, IMU, pitch/roll, terrain/context.
- Core: M2/M3 physics.
- Residual: learned correction for speed and yaw rate.
- Constraint: residual must be evaluated against held-out sessions and never hide bad ground truth.

## Dataset Quality Checks

For a session to be research-grade:

- `canonical_quality.status == PASS`
- ICP coverage > 0.95 for normal bags.
- `max_icp_gap_s < 0.2`.
- `p99_pose_step_m < 0.30`.
- `map_freeze_detected == false`.
- `has_real_tacho` coverage high when testing wheel models.
- IMU coverage high when testing yaw-rate fusion.
- No large timestamp jumps.
- Visual map/traj overlay has no obvious double walls.

## Visual Inspection

Generate a visual report:

```bash
python3 scripts/visualize_canonical_dataset.py data/<session>
```

Important plots:

- `map_trajectory_overlay.png`: map VTK + ICP trajectory + MTT odom + optional aligned ZED odom.
- `signals_motion_inputs.png`: command, tachometer, ICP/odom speed, IMU yaw rate, articulation.
- `motion_model_current_baseline.png`: current fitted model against ICP.

VTK files are ASCII POLYDATA. They can be opened in CloudCompare directly. For Python plots, read the `POINTS N float` block and plot X/Y; the report script does this with downsampling for large maps.

## Recommended Workflow

1. Build canonical ICP/map for all bags.
2. Audit all datasets.
3. Generate research visual reports.
4. Fit M0/M1/M2/M3 on selected training sessions.
5. Validate on held-out sessions.
6. Only then use the model for WILN/controller tuning.

Commands:

```bash
python3 scripts/audit_postprocess_dataset.py data
python3 scripts/fit_motion_model_from_bags.py data
python3 scripts/visualize_canonical_dataset.py data
```

