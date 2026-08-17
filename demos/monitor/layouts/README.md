# MTT Foxglove layouts

| File | What it is |
|---|---|
| `mtt_command_center.json` | Full multi-tab command & monitoring layout (see below) |
| `mtt_dashboard_cloud.json` | Export of the `mtt_dashboard` layout from the Foxglove account |
| `default_cloud.json` | Export of the `Default` layout from the Foxglove account |

The three `demos/*/mtt_dashboard.json` files are byte-identical copies of one
older single-screen layout.

## `mtt_command_center.json` — command center

### Import

Foxglove Desktop: **Layouts → Import from file…** → this file. Connection:
`ws://localhost:8766` (via `demos/monitor`, recommended) or
`ws://192.168.2.2:8765` (robot bridge, robot Wi-Fi) or
`ws://192.168.50.2:8765` (Doodle field mode).

### Tabs

| Tab | Contents |
|---|---|
| 🎮 Drive | Robot-following 3D (Hesai + ICP map + trajectory), MODE / E-STOP indicators, MANUAL / AUTO / STOP buttons, speed gauge, cmd-vs-odom plot, ZED camera |
| 🛤️ Teach & Repeat | Map + plans 3D, ROUTE READY indicator, teach/replay/follower/supervisor states, supervisor-checked buttons (see wiring below), save/load route |
| 🚜 Trailer | RS-Airy cloud + trailer markers, articulation gauge/plots (encoder vs LiDAR vs driver), trailer pose |
| ❤️ Health | E-stop, deadman, speed, temperature, `/mtt_health`, `/mtt_status`, battery, IMU, ROS logs, factor-graph innovation z-scores |
| 🛰️ GPS Teach & Repeat | `mtt_gps_teach_repeat` — see below |
| 📈 GPS Control | Frenet tracking errors, articulation/curvature/speed commands, route progress and obstacle slowdown |
| ⚙️ Tuning | **Parameters** panel (live ROS parameter edit) + raw WILN command sender |

### Button wiring (verified against the composes)

Every button maps 1:1 to what the `demos/live_robot` compose helpers run.
All Teach & Repeat buttons go through the **repeat supervisor**, which enforces
the safety checks (ICP watchdog, mode, route grading) — same as the CLI:

| Layout button | Calls | Compose equivalent |
|---|---|---|
| MANUAL / AUTO / STOP | `/mtt_control/request_{manual,auto,stop}` (Trigger) | mode manager services |
| ⏺️ START TEACH | `/mtt_repeat/teach_start` | `dc run --rm wiln_teach_start` |
| ⏹️ STOP TEACH | `/mtt_repeat/teach_stop` | `dc run --rm wiln_teach_stop` |
| ✅ MARK READY | `/mtt_repeat/mark_ready` | second half of `dc run --rm wiln_load` |
| ▶️ REPLAY ROUTE | `/mtt_repeat/play_line` | `dc run --rm wiln_replay` / `route_replay` |
| 🔁 REPLAY LOOP | `/mtt_repeat/play_loop` | `dc run --rm wiln_replay_loop` |
| ✖️ CANCEL REPLAY | `/mtt_repeat/cancel` | `dc run --rm wiln_replay_stop` |
| 💾 SAVE ROUTE | publish `save:<path>` on `/wiln/command` | `dc run --rm wiln_save` |
| 📂 LOAD ROUTE | publish `load:<path>` on `/wiln/command` | first half of `dc run --rm wiln_load` |

Required stack: `live_robot` with the **`wiln` profile** up (which requires
`mapping`). The field-light bridge whitelist already allows `/mtt_.*`,
`/wiln/.*` and `/mapping/.*` services, so all buttons work over Doodle too.

Safety facts (from `wiln.yaml` / supervisor config):
- The follower has `require_deadman: true` — REPLAY arms the route, but the
  robot only moves while the operator holds the physical deadman (RB).
- Replay requires AUTO mode; the supervisor requests it and watches ICP,
  teleop override, and health timeouts, and auto-disarms after 5 min.
- SAVE/LOAD default to `data/wiln_routes/demo_route/route.ltr`; the compose
  helpers compute the real dated path via `wiln_route_env.sh` — edit the panel
  value, or keep using `dc run wiln_save` for auto-named routes.

### Parameter tuning — what is actually live

The bridge exposes `parameters` + `parametersSubscribe`, so the Tuning tab can
set any parameter on any node immediately. **But most MTT/WILN nodes read
their parameters once at startup** (only `com_position_node` registers a
parameter callback), so a live set often has no effect until the node restarts.

Practical rule:
- **Live-effective**: panel display options, anything read per-cycle by a node.
- **Restart the service after editing YAML** (`demos/common/config/*.yaml`):
  speed/accel limits (`mtt_control.yaml: max_linear_speed`, slew rates
  `linear_rise_rate`…, `mtt_driver.yaml: mtt_can_node.max_linear_speed_ms`),
  WILN speeds (`wiln.yaml: default_speed_ms`, `max_speed_ms`,
  `trajectory_speed`), supervisor timeouts.
- **Container-start only**: everything in `runtime.env` (`ENABLE_*`,
  `MAPPING_*`, `FIELD_NETWORK_MODE`, voxel debug settings) — needs
  `docker compose up -d --force-recreate <service>`.

### Network / field mode

- Robot Wi-Fi (`norlab_mtt`): connect Foxglove to `ws://192.168.2.2:8765`, or
  run `demos/monitor` and use `ws://localhost:8766`.
- Doodle: set `FIELD_NETWORK_MODE=true` (robot `runtime.env` + monitor env) —
  bridge binds to `192.168.50.2`, monitor targets the Doodle endpoint, and the
  robot switches to the **field-light whitelist**
  (`foxglove_bridge_field_light.yaml`): raw `/hesai_lidar/points` and images
  are NOT sent; use `/debug/hesai_points_voxel` (already in the 3D panels,
  toggle it on) and the 10 MB send buffer keeps latency bounded.
- Pre-flight checks: `scripts/field_doodle_check.sh`,
  `scripts/audit_network_field.sh`, `scripts/pre_session_check.sh`, and
  `docker compose --profile check up health_check` in `demos/data_collection`.
- Docs: `docs/reference/networking.md`, `docs/reference/doodle_field_handoff.md`.

### 🛰️ GPS Teach & Repeat tab

Backed by the new `mtt_gps_teach_repeat` package (`src/mtt_core/`) — GPS-only
teach and repeat, independent of WILN, same AUTO+deadman safety contract. Full
details: `src/mtt_core/mtt_gps_teach_repeat/README.md`.

The backing nodes and `localization` now start by default with `dc up -d`.
The follower remains disarmed
until the operator presses GPS REPLAY and all AUTO/deadman/match/obstacle
gates are valid.

| Panel | Frame / topics | Notes |
|---|---|---|
| 3D | `map` frame — GPS reference (blue), local horizon (yellow), live Teach (green), Replay actually driven (magenta), ICP session trajectory (grey), robot/trailer markers and front obstacle points (red) | `full_path` is the fitted route; `local_horizon` is what the follower tracks; `/mtt/gps_path_follower/executed_path` is cleared at each GPS Replay and only records while that Replay is armed |
| Map (satellite) | `/gps_front/fix` | front antenna fix, historyMode "all" — situational awareness, not the control-loop pose |
| Indicators | fix quality (`/mtt/gps_recording/fix_quality`, color-coded RTK FIX/FLOAT/DGPS/SPP/NO FIX) + path match status (`/mtt/gps_path_server/diagnostics`) |
| Timeline | Teach, path matching, route-loaded gate and follower lifecycle in one synchronized `StateTransitions` panel |
| Gauges | horizontal GPS accuracy, signed commanded speed, remaining route distance and live obstacle slowdown factor |
| Buttons | GPS Teach/Stop/Clear, Replay/Stop, List/Load/Status, and editable live AUTO speed cap |
| GPS Control tab | lateral/course Frenet error, desired curvature, articulation command, signed speed, remaining distance and slowdown (`/mtt/gps_path_follower/diagnostics`) |

Recording quality controls (configurable in `gps_recorder.yaml`): fix-quality
gating, GPS-jump rejection, and post-recording smoothing — see the package
README for the exact thresholds and rationale.

GPS Replay is protected by the front LiDAR stop/slowdown gate. Path
deformation around an obstacle remains WILN-only for now.

The Doodle field-light whitelist (`foxglove_bridge_field_light.yaml`) now also
allows `/localization/.*` so the GPS tab's pose arrow works in field mode.

### ❤️ Health tab — factor-graph innovation plot

`Plot!innovation` (bottom-right, next to the ROS log) plots four z-scores
from `/localization/factor_graph/innovation_diagnostics`
(`std_msgs/Float64MultiArray`, published by `factor_graph_node` every
optimizer tick): GPS position (`.data[2]`), articulation yaw (`.data[5]`),
LiDAR-ICP odom (`.data[9]`), trailer LiDAR pose (`.data[13]`) — each is the
measurement's innovation whitened by the noise sigma actually assigned to
that factor (not a full NEES against the joint covariance — see
`docs/roadmap/POSE_ESTIMATION_PLAN.md` P2.1). A channel sitting consistently
near 0 most of the time with occasional legitimate spikes is healthy; one
sitting persistently high means that factor's noise model is too tight for
what the sensor is actually delivering (this is exactly the shape the GPS
Fix/Float bug — P0.1 in the same doc — would have shown up as, had this
existed sooner). Each channel also carries a `_valid` flag (`.data[0]`,
`[3]`, `[6]`, `[10]`) — LiDAR-odom and trailer-pose specifically only report
a number when a clean, no-lag factor was actually added that tick.

### Point cloud compression

The Cloudini Foxglove extension is installed on this PC, but nothing in the
stack publishes Cloudini-compressed clouds — the bandwidth strategy is the
voxel debug cloud + `MAPPING_MAP_PUBLISH_RADIUS_M` crop + field-light
whitelist. If Doodle bandwidth ever becomes the bottleneck for clouds, adding
a `pointcloud_republisher` with Cloudini encoding on the robot would let the
extension decode it for free on the laptop.

### Safety

Buttons publish real commands to the robot. **STOP** (`request_stop`) is the
first reflex; the physical e-stop remains the final authority.
