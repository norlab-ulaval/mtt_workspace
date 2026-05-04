# MTT CAN tools

This folder contains the CAN files used to inspect, decode, and export MTT bus data.

It is meant for local debugging and log analysis.

## Files

- `MTT_CAN_v1_1_simple.dbc`
- `mttDriver.dbc`
- `../scripts/mtt_can_monitor.py`
- `../scripts/mtt_can_audit.py`
- `../scripts/mtt_can_export.py`

Recommended usage:

- `MTT_CAN_v1_1_simple.dbc`
  curated local DBC, aligned with the current ROS driver semantics
- `mttDriver.dbc`
  raw DBC recovered from the robot-side scripts, useful as an additional source
  of truth and for checking signal naming or scaling differences

## Covered frames

The current DBC covers these frames:

- `0x001`
- `0x100`
- `0x2FF`
- `0x300`
- `0x301`
- `0x600`
- `0x601`
- `0x602`
- `0x603`

The most useful battery frame is `0x602`.

It exposes:

- state of charge
- battery current
- battery voltage
- heatpad state
- remaining charge time

## Temperature map

The current decoded temperature signals are:

- `0x2FF`
  - `MainSensorTempA`
  - `MainSensorTempB`
  - main controller / main module temperatures
- `0x600`
  - `CellTemp1` to `CellTemp4`
  - battery cell-group temperatures
- `0x601`
  - `AmbientTemp`
  - `MosfetTemp`
  - `HeatpadATemp`
  - `HeatpadBTemp`
  - battery enclosure / BMS / heating temperatures

## Raw vs physical values

The driver now follows a strict rule:

- `*_raw` = direct CAN payload
- physical values are only published when the scaling is actually known

At the moment:

- battery temperatures are treated as physical temperatures from the local CAN spec
- battery current is also published as an estimated ampere value using the DBC scaling you provided
- battery voltage is still kept as raw until validated on the robot
- power in watts is intentionally kept invalid until voltage scaling is confirmed

## Current limits

Two points still need robot-side validation:

- the final role of `0x001` versus `0x100`
- the physical scaling of some battery values

Because of that, some fields are kept as raw values on purpose. Raw data is better than a wrong unit.

## Health topic and fallback odom

The ROS driver now exposes a higher-level monitor topic:

- `mtt_health`

It is meant for operator checks and bagging diagnostics:

- command currently sent
- telemetry freshness / tachometer stale state
- main module temperatures
- battery / BMS temperatures
- raw vs estimated current / voltage / power semantics
- simple warnings such as brake+throttle conflict

It also publishes:

- `mtt_monitor/cmd_fallback_odom`

This is a command-only motion estimate used only as a degraded fallback when the
tachometer is stale or broken. It is not a replacement for LiDAR/ICP, GNSS, or
any other real localization source.

## Temperature interpretation used by the monitor

The terminal monitor labels the temperature sources conservatively:

- `MainSensorTempA`
  main controller / main module side
- `MainSensorTempB`
  encoder / tachometer side to watch closely during experiments
- `CellTemp1..4`
  battery cell-group temperatures
- `AmbientTemp`
  battery enclosure ambient temperature
- `MosfetTemp`
  battery power electronics temperature
- `HeatpadATemp`, `HeatpadBTemp`
  battery heating pad temperatures

## Usage

Run these commands from the repository root.

If `cantools` is already available in your shell:

```bash
python3 scripts/mtt_can_monitor.py --interface can0
python3 scripts/mtt_can_audit.py --interface can0 --duration 8
python3 scripts/mtt_can_export.py mtt_session.log
```

If you prefer to use the local tool environment shipped with this repository:

```bash
.venv-tools/bin/python
```

### Live monitor

```bash
.venv-tools/bin/python scripts/mtt_can_monitor.py --interface can0
```

### Passive audit

```bash
.venv-tools/bin/python scripts/mtt_can_audit.py --interface can0 --duration 8
```

### Export a `candump` log

Capture:

```bash
candump -L can0 > mtt_session.log
```

Export:

```bash
.venv-tools/bin/python scripts/mtt_can_export.py mtt_session.log
```

This produces:

- `mtt_session.jsonl`
- `mtt_session.csv`

To keep only the main MTT frames:

```bash
.venv-tools/bin/python scripts/mtt_can_export.py mtt_session.log \
  --ids 0x001 0x100 0x2FF 0x600 0x601 0x602 0x603
```
