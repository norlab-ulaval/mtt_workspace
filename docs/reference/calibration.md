# Calibration and geometry

Calibration values are metres and radians. The source files are
`src/mtt_core/mtt_description/urdf/calibrations/calib_v1.xacro` and
`calib_v2.xacro`. The main `robot.urdf.xacro` consumes all sensor properties.

## Selecting a calibration

| Workflow | Default | Selection |
|---|---|---|
| Live robot, data collection, COM demo | v2 | `CALIBRATION=v1` or `CALIBRATION=v2` |
| Bag replay | v1 | select the mounting state recorded in the session |

The description launch reads calibration directly from source using
`calib_dir`, so editing source changes the next launch even without rebuilding
the installed package. Record the calibration file hash with each session.
Historical notes associate v1 with bags through 2026-06-24 and v2 with the
subsequent cage relocation; dates alone do not establish the actual mounting.

## Current values and unresolved measurement check

| Quantity | v1 | Current v2 |
|---|---|---|
| Hesai translation in `reference_point` | (0.620000, -0.266000, 0.355000) | (0.850829, -0.266000, 0.355000) |
| Hesai yaw | pi/2 | pi/2 |
| IMU, MTi-10, ZED mounts in Hesai frame | original mounts | same as v1 |
| RS-Airy mount in Hesai frame | April calibration | separate June calibration |

The 2026-08-27 field note in v2 reports restoring the original Hesai yaw and
cage-internal mounts after an erroneous pi-yaw conversion. This audit preserves
those numbers. The RS-Airy values are separately measured, not inferred by
rotating the v1 values.

**The direction of cage displacement still needs confirmation.** The original field note
described a backward movement, but v2 minus v1 is **+0.230829 m in X**.
`reference_point_joint` has zero rotation relative to `base_link`, whose X
points forward. The encoded movement is therefore forward. Do not reverse the
sign solely to match the comment: compare the measurement and original static
bag first. The recorded RS-Airy vertical component is also marked as poorly
constrained; successful XML generation is not calibration accuracy evidence.

## Generating a candidate

`scripts/calib_hesai_scan.py` estimates position/tilt from a static scan. Its
historical crop masks assume raw X-forward/Y-left points, while the named
Hesai mounting convention is X-left/Y-backward. Verify the input cloud basis
and any driver rotation before interpreting its estimates. No new numerical
calibration was performed during the handover audit.

```bash
mkdir -p artifacts/calibration
python3 scripts/calib_hesai_scan.py /path/to/static_bag \
  --template src/mtt_core/mtt_description/urdf/calibrations/calib_v2.xacro \
  --wall-dist 1.20 --output artifacts/calibration/candidate.xacro
```

Output is a complete, explicitly unvalidated candidate. Existing output files
are refused. The template yaw and all other sensor mounts are preserved;
unestimated position/tilt components keep their template values. Compare the
candidate against the original, expand Xacro, and validate cloud alignment
before installing it as a named calibration.

## Geometry used by different algorithms

The rear obstacle monitor uses hitch `(-1.45, -0.085, 0.35)` in `base_link`,
trailer rear length 1.90 m and `yaw_prior = pi - theta`, matching its declared
trailer-pose geometry. It consumes `/trailer/articulation_angle`.

Research models use different historical parameter sets: the recovered
`scripts/lib/mtt_motion_research.py` uses nominal distances 0.9/1.5 m, whereas
`scripts/analyze_ice_session.py` uses audited distances 0.834227/1.5135 m.
Do not silently replace one with the other or compare results without recording
which geometry, sign convention and articulation source produced them.
