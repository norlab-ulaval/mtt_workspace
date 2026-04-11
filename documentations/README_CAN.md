# MTT CAN tools

This folder contains the CAN files used to inspect, decode, and export MTT bus data.

It is meant for local debugging and log analysis.

## Files

- `MTT_CAN_v1_1_simple.dbc`
- `../scripts/mtt_can_monitor.py`
- `../scripts/mtt_can_audit.py`
- `../scripts/mtt_can_export.py`

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

## Current limits

Two points still need robot-side validation:

- the final role of `0x001` versus `0x100`
- the physical scaling of some battery values

Because of that, some fields are kept as raw values on purpose. Raw data is better than a wrong unit.

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
