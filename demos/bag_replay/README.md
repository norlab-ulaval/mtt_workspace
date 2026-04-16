# bag_replay

This demo replays a recorded MCAP bag against the MTT stack with bag time.

## Basic use

From `demos/bag_replay`:

```bash
BAG_PATH=/path/to/data/mtt_nominal_2026-04-15_15-41-53 docker compose up
```

That starts:
- `bag_player`
- `description`
- `foxglove`

Open Foxglove Studio separately and connect to `ws://localhost:8765`.

## Optional profiles

Replay with localization:

```bash
BAG_PATH=/path/to/bag docker compose --profile localization up
```

Replay with perception:

```bash
BAG_PATH=/path/to/bag docker compose --profile perception up
```

Slow the replay down when debugging:

```bash
BAG_PATH=/path/to/bag REPLAY_RATE=0.5 docker compose up
```

## Notes

- `use_sim_time` is handled by the replay-side launch paths.
- If the bag comes from `demos/data_collection`, the session metadata is printed at startup.
- Raw sensor packets stay useful here because replay is not limited to the already-processed topics.
