# demos

This directory is reserved for reproducible local scenarios and launch recipes.

Run these from the repository root so Compose picks up the workspace `.env`:

```bash
xhost +local:docker
docker compose --env-file .env -f demos/description/compose.yaml up description
docker compose --env-file .env -f demos/simulation/compose.yaml up simulation
docker compose --env-file .env -f demos/monitor/compose.yaml up monitor_demo
```

The root services remain available too:

```bash
docker compose run --rm bash
docker compose run --rm compile
docker compose up monitor
```
