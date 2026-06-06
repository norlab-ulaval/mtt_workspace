# simulation

Local Gazebo + joystick + motion-model tuning entry points.

## Main workflow

Edit gains and test shape in:

```bash
demos/simulation/config/tuning.env
```

Then run:

```bash
dcup -d simulation control path_follow model_monitor rviz
```

Launch one trajectory:

```bash
dcu path_arc
dcu path_s_curve
dcu path_straight
```

If you changed follower gains in `config/tuning.env`:

```bash
dcup -d --force-recreate path_follow
```

If you changed Gazebo bridge signs or speed scale:

```bash
dcup -d --force-recreate simulation
```

## Services

- `simulation`: Gazebo + simulated robot runtime.
- `control`: same joystick/control stack as the real robot.
- `path_follow`: `mtt_path_follower` on Gazebo ground truth `/mtt_odometry/ground_truth`.
- `model_monitor`: compares `/mtt_odometry` against Gazebo ground truth so the simulated motion model can be tuned instead of trusted blindly.
- `path_arc`: one-shot arc trajectory from current robot pose.
- `path_s_curve`: one-shot S-curve trajectory from current robot pose.
- `path_straight`: one-shot straight trajectory from current robot pose.
- `rviz`: motion-model tuning view.
- `foxglove`: Foxglove bridge.

## RViz

The tuning RViz view shows:

- blue: `/sim_motion_model/test_path`
- green: `/mtt_path_follower/reference_path`
- orange: `/mtt_path_follower/target_pose`
- robot model, TF, and `/mtt_odometry/ground_truth`

## Debug topics

```bash
/mtt_path_follower/debug/lateral_error_m
/mtt_path_follower/debug/heading_error_rad
/mtt_path_follower/debug/kappa_desired_m_inv
/mtt_path_follower/debug/kappa_feedforward_m_inv
/mtt_path_follower/debug/kappa_adaptive_bias_m_inv
/mtt_path_follower/debug/kappa_command_m_inv
/mtt_path_follower/debug/kappa_effective_est_m_inv
/mtt_path_follower/debug/psi_cmd_rad
/mtt_path_follower/debug/slip_scale
/sim_model_monitor/position_error_m
/sim_model_monitor/heading_error_rad
/sim_model_monitor/distance_ratio_model_over_gt
/sim_model_monitor/status
/mtt_tachometer
/mtt_health
/mtt/articulation_state
/mtt_articulation_angle
```
