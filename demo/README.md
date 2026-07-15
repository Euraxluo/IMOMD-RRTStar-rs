# IMOMD-RRT* V2X Demo

Interactive FastAPI + Canvas demonstration for the Rust reproduction of
IMOMD-RRT\*. The UI overlays dynamic edge weights on a road graph and drives a
pluggable `NavigationSession` that races greedy / exact / IMOMD solvers, streams
anytime improvements, warm-starts after traffic updates, and reseeds from an ego
pose.

The **web UI is English-only**.

## Launch

```bash
# Build the Python extension from the repository root
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  .venv/bin/maturin develop --release --features python,extension-module

./demo/run.sh
# http://127.0.0.1:8000
```

Default map preference: `tmp/imomd-cpp/osm_data/FRB2.osm` (override with
`DEMO_OSM_PATH` / `DEMO_MAP`). Missing OSM falls back to the bugtrap fake map.

## Basic interaction

1. Stay in **Pick route** mode.
2. Click near a road for the **start** (green).
3. Click one or more **waypoints** (orange), then **Waypoints done — pick goal**.
4. Click the **goal** (pink); planning starts automatically.

Clicks snap to nearby graph nodes. WebSocket heartbeats do not reset an in-progress
selection. Route legs use distinct colors (cyan then pink for the first two legs).

**Smart route** picks a spread reachable start / waypoints / goal automatically.

### Anytime / ego / plugins

- **Anytime ON**: background `continue_search`; path and cost chart improve over time.
- **Set ego**: replan the remaining trip from the snapped node.
- **Race lanes**: `greedy` / `exact` (≤8 waypoints) / `imomd` run together; the first
  feasible path is shown, then only strictly better costs replace it. See
  [docs/navigation-plugins.md](../docs/navigation-plugins.md).

### Map scenarios

| key | Description |
|---|---|
| `chicago_mega` | Chicago-style mega grid (~5000 nodes) |
| `chicago_osm` | Real downtown OSM when downloaded locally |
| `city_large` | Medium synthetic city (good for API smoke) |
| `osm_or_fake` / `bugtrap` | OSM or small regression map |

```bash
python scripts/download_chicago_osm.py
DEMO_MAP=chicago_osm ./demo/run.sh
```

## Traffic and V2X

In **Draw traffic**, outline a polygon (≥3 points), choose a level, then **Apply**:

| Level | Weight |
|---|---|
| `free` | ×1 (restore) |
| `slow` | ×2.5 |
| `jam` | ×5 |
| `blocked` | edge removed |

Auto V2X emits a zone event every ~3s from a single server-side scheduler and
broadcasts one shared state to all WebSocket clients. Traffic updates warm-start
IMOMD trees; only destination changes fully restart the race.

## Verification signals

Each snapshot runs an independent check (endpoints, edges, cost arithmetic,
Dijkstra+TSP oracle when the waypoint count is small). **Verify ✓** means the
contract holds. IMOMD is anytime and may sit above the oracle temporarily.

## API

- `GET /api/state` — map, path, verification, events, warm-start stats
- `GET /api/verify` — verification report
- `POST /api/destinations` — `{source, objectives, target}`
- `POST /api/destinations/auto`
- `POST /api/traffic/edge` · `/edges` · `/zone` · `/clear`
- `POST /api/replan?seconds=1.5`
- `POST /api/v2x/auto?enabled=true|false`
- `POST /api/anytime?enabled=true|false`
- `WS /ws/v2x`

## Tests

```bash
python scripts/test_demo_api.py          # server must be running
uv run --project demo --group dev pytest demo/tests -v
```
