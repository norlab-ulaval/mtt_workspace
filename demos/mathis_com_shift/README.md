# mathis_com_shift

Robot-side data collection profile for Mathis COM-shift experiments.

This demo is intentionally separate from `demos/data_collection` so Mathis can tune
mapper and COM motor settings without changing the trailer-oriented field config.

## What Is Different

- No trailer by default: `ENABLE_PERCEPTION=false`, `ENABLE_CLOUD_MERGER=false`.
- Mapper config is local: `config/mapper_snow_no_trailer.yaml`.
- COM motor config is local: `config/com_motor.yaml`.
- Control config is local: `config/mtt_control.yaml` with 20 km/h teleop ceiling
  and more aggressive acceleration only for this demo.
- Health check validates COM motor topics when `COM_MOTOR_REQUIRED=true`.
- Optional C++ ice slip detector fuses ICP + tachometer.
- The `ice` profile arms a Mathis-only transient experiment: full-speed command
  flips direction when slip settles, and COM command flips to the opposite limit.
- `imu_odom_mti100` is the default odom TF authority for this demo, matching the
  Mathis/William mapper path. The MTT driver still publishes `/mtt_odometry`, but
  `ODOMETRY_BROADCAST_TF=false` avoids a duplicate `odom -> base_footprint` TF.

## Normal Flow

Run from this directory:

```bash
cd demos/mathis_com_shift
docker compose --profile check up health_check
```

Start the live stack and recorder:

```bash
SESSION_TYPE=com_shift \
TRAILER_ATTACHED=false \
TERRAIN=neige \
OPERATOR=Mathis \
GPS_MODE=serial \
docker compose --profile record up
```

The default stack starts driver, description, sensors, mapping, COM motor,
Foxglove, Zenoh, socketcan bridge, and the recorder profile.

## Mapper Tuning

Primary files for this demo:

```text
demos/mathis_com_shift/config/runtime.env
demos/mathis_com_shift/config/mapper_snow_no_trailer.yaml
```

The local mapper YAML removes the trailer self-filter volume so rear snow/terrain
points are kept. Runtime launch parameters in `runtime.env` also disable dynamic
trailer filtering:

```text
MAPPING_ENABLE_DYNAMIC_TRAILER_SELF_FILTER=false
MAPPING_CONFIG=demos/mathis_com_shift/config/mapper_snow_no_trailer.yaml
```

The mapper waits for the real TF `odom -> base_footprint` before starting. In
this demo that TF comes from `imu_odom_mti100`, not from the MTT odometry node.
Altitude support is compiled in through `src/external/imu_odom@altimeter-icp`,
but the Mathis profile keeps `IMU_ODOM_USE_ALTITUDE=false`.

## COM Motor

Local motor config:

```text
demos/mathis_com_shift/config/com_motor.yaml
```

Useful checks:

```bash
docker compose --profile check up health_check
docker compose logs -f com_motor
docker compose run --rm bash
ros2 topic echo /motor/driver_status --once
ros2 topic hz /motor/joint_state
```

Set `COM_MOTOR_DRY_RUN=true` in `config/runtime.env` to test the ROS command
chain without EtherCAT hardware.

Switch COM motor direction live:

```bash
# Invert right-stick to COM motor direction
docker compose run --rm bash ros2 topic pub --once \
  /mtt_control/com_direction_sign std_msgs/msg/Float64 "{data: -1.0}"

# Return to normal
docker compose run --rm bash ros2 topic pub --once \
  /mtt_control/com_direction_sign std_msgs/msg/Float64 "{data: 1.0}"
```

The switch resets the current COM steer command to zero. The next joystick update
continues with the selected sign, still gated by the deadman.

Joystick controls in this demo:

```text
Button 7 short press       COM mode ON/OFF
Button 7 medium press      COM set_home
Button 7 long press        COM park / return home
Right stick                COM steer only when COM mode is ON
Deadman                    still required for motion commands
```

`config/mtt_control.yaml` is Mathis-only. It sets `max_linear_speed: 5.56`
(`20 km/h`) and `linear_rise_rate: 3.0`; the common trailer config stays at
`4.2 m/s` (`15 km/h`).

## Ice Slip Diagnostics

Start slip diagnostics with the live stack:

```bash
docker compose --profile ice up ice_slip_detector
```

Start the full Mathis transient experiment profile:

```bash
docker compose --profile ice up
```

Experiment behavior:

```text
1. The ice profile publishes /mtt_control/ice_com_shift_experiment_active.
2. Hold deadman.
3. Push the linear stick past 70% once to choose the first direction.
4. COM PARK/home is requested for 0.15s before the first movement, so COM starts
   near home instead of wasting the first slip window crossing the full travel.
5. The node commands +/-5.56 m/s through the manual command path.
6. COM mode is forced ON for the experiment and COM steer is sent to +1/-1.
7. When abs(/ice_slip/slip_ratio) stays below 0.08 for 0.10s, speed and COM
   limit both flip sign. The COM target flips immediately while the speed filter
   is decelerating/reversing the robot.
8. While holding a side, COM jitters around that side by default: 100% -> 75%
   -> 100% at 4 Hz. This adds repeated inertial kicks without leaving the active
   side. Set `ice_com_shift_jitter_enabled: false` to disable it, or
   `ice_com_shift_jitter_amplitude: 1.0` for full home-to-limit pulses.
9. Pressing the brake above 10% also requests a one-shot phase switch; release
   the brake before another brake-triggered switch can happen.
10. Releasing deadman stops and resets the experiment state, and requests COM
   PARK/home. It does not drive COM to the opposite limit without deadman.
```

To keep only slip diagnostics without arming the experiment:

```bash
ICE_COM_SHIFT_EXPERIMENT_ARM=false docker compose --profile ice up
```

Check outputs:

```bash
ros2 topic echo /ice_slip/slip_detected --once
ros2 topic echo /ice_slip/slip_ratio --once
ros2 topic echo /ice_slip/icp_slip_ratio --once
ros2 topic echo /ice_slip/body_speed_ms --once
ros2 topic echo /ice_slip/fused_speed_ms --once
ros2 topic echo /ice_slip/tacho_speed_ms --once
ros2 topic echo /ice_slip/cmd_driver_delta_ms --once
ros2 topic echo /ice_slip/com_direction_sign --once
ros2 topic echo /mtt_control/ice_com_shift_armed --once
ros2 topic echo /mtt_control/ice_com_shift_active --once
ros2 topic echo /mtt_control/ice_com_shift_phase --once
```

The detector reads timestamped `/cmd_vel`, `/mapping/icp_odom`, `/mtt_tachometer`
and `/mtt_status`. Slip is computed from command speed versus a 1D fused speed:
ICP is the strong measurement, tachometer is weak by default
(`ICE_SLIP_TACHO_SPEED_VARIANCE=1.00`) because the encoder is noisy and may be
synthetic. `/ice_slip/cmd_driver_delta_ms` compares final `/cmd_vel` to the
effective command reported by the CAN driver.

The transient experiment is implemented inside `mtt_operator_input_node`, not as
a second publisher on `/cmd_vel`. It therefore still goes through the existing
manual filter and arbiter. The Mathis config sets the manual filter ramp to
`20.0 m/s^2` and disables the reversal guard only in this demo, because the goal
is to create slip on ice. `com_position_node.park_resume_on_steer=true` is also
enabled only in this demo so PARK can return to SETUP when the experiment sends
the next non-zero COM command. COM jitter topics are recorded through
`/mtt_control/com_steer` and `/motor/cmd_position`. Do not use this profile on
high-traction ground.

The diagnostic topics are appended to the recording topic list through
`EXTRA_RECORD_TOPICS` in `config/runtime.env`.

## IMU Odom

`imu_odom_mti100` starts with the normal stack. It subscribes to `/mti100/data`
whose TF frame is `imu_link`, subscribes to `/mapping/icp_odom`, publishes `/imu_odom`, and broadcasts
`odom -> base_footprint`.

Keep exactly one odom TF source:

```text
ODOMETRY_BROADCAST_TF=false
IMU_ODOM_USE_ALTITUDE=false
MAPPING_ROBOT_FRAME=base_footprint
```

`base_footprint` is the planar robot base used by the driver/odometry. `base_link`
is the physical body frame, fixed 10 cm above `base_footprint` in this robot.
Mapper pose/odom should use `base_footprint`; self-filter bounding boxes stay in
`base_link` through `filtering_frame=base_link`.
