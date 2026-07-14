# Realtime Pluggable Navigation Runtime — Design

**Date:** 2026-07-14  
**Status:** Approved for implementation (approach C: thin Rust core + demo client together)  
**Context:** Evolve IMOMD-RRTStar-rs from one-shot V2X replans into an event-driven, algorithm-pluggable realtime navigation session that can show anytime path improvement and absorb traffic/ego changes.

## Goals

1. **Dynamic road costs** — traffic overlays mutate edge weights without rebuilding topology identity.
2. **Realtime plan streaming** — clients receive incremental `PlanUpdate`s (path / cost / reason) instead of only a final blocking result.
3. **Ego-driven replan** — user/vehicle position changes become a first-class event; search restarts from the snapped road node when needed.
4. **Pluggable planners** — IMOMD-RRT* is one `PlannerPlugin`; LPA* / D* Lite / CCH can later implement the same trait and drop into the same session + demo.

## Non-goals (v1)

- Implementing LPA* / D* Lite / CCH algorithms themselves.
- ROS / production map matching / GPS noise models.
- Bit-for-bit C++ parity for streaming UI.
- Multi-vehicle fleets.

## Architecture

```
EventSources → NavigationSession → PlannerPlugin → PlanUpdate stream → Gateway → Clients
                 ↓
           RoadNetworkStore (topology + overlay revision)
```

### Components

| Unit | Responsibility |
|---|---|
| `DomainEvent` | `TrafficChanged`, `EgoMoved`, `DestinationsSet`, `ContinueSearch` |
| `RoadNetworkStore` | Holds `Arc<AdjacencyGraph>` materialization + revision counter |
| `PlannerPlugin` | `id`, `reset`, `continue_search`, `on_graph_changed`, `on_ego_moved`, `best` |
| `ImomdPlugin` | Adapter over existing `ImomdRrtStar` (`step` / `run_for` / `update_graph`) |
| `NavigationSession` | Applies events, chooses fresh/warm/ego-reseed, emits `PlanUpdate`s |
| Demo Gateway | FastAPI WS broadcasts `plan_update` / `state` |
| Demo UI | Live path morph + cost timeline + ego marker + algorithm selector |

### `PlannerPlugin` contract (v1)

```rust
trait PlannerPlugin {
    fn id(&self) -> &'static str;
    fn reset(&mut self, graph: Arc<AdjacencyGraph>, destinations: Destinations) -> Result<()>;
    fn on_graph_changed(&mut self, graph: Arc<AdjacencyGraph>) -> Result<GraphUpdateStats>;
    fn on_ego_moved(&mut self, ego_node: NodeId, remaining: Destinations) -> Result<()>;
    fn continue_search(&mut self, budget: Duration) -> Result<Vec<PlanUpdate>>;
    fn best(&self) -> Option<&PlanningResult>;
}
```

`on_ego_moved` default for IMOMD: rebuild destinations with `source = ego_node`, keep remaining objectives/target, fresh or warm as plugin decides (IMOMD v1: fresh reset with new source).

### `PlanUpdate`

```rust
struct PlanUpdate {
    sequence: u64,
    reason: UpdateReason, // Expanded | Improved | Connected | Finished | EgoReseed | TrafficWarmStart
    path: Option<Vec<NodeId>>,
    cost: Option<f64>,
    visit_order: Option<Vec<usize>>,
    explored_nodes: Option<usize>,
    replan_mode: &'static str,
    tree_update: Option<GraphUpdateStats>,
}
```

Streaming rule for IMOMD: during `continue_search`, call `step()` in a loop until budget; emit an update when status is `Connected`, `PathImproved`, or `Finished` (and optionally throttle `Expanded`).

### Session event handling

| Event | Session action |
|---|---|
| `DestinationsSet` | `plugin.reset`, then `continue_search` |
| `TrafficChanged` | bump revision, materialize graph, `plugin.on_graph_changed`, `continue_search` |
| `EgoMoved` | snap lat/lon → nearest node; if off active path (or always), `plugin.on_ego_moved`, `continue_search` |
| `ContinueSearch` | `continue_search` only (anytime improve) |

### Demo UX (v1)

- Keep existing map / traffic / V2X.
- Add **Anytime** toggle: background slices of `ContinueSearch` push improving paths.
- Add **Ego** control: click “设为当前位置” on a node, or auto-advance along path for demo.
- Show **cost history** sparkline from streamed improvements.
- Algorithm dropdown: `imomd` (live) + disabled placeholders `lpa_star` / `d_star_lite`.

## Testing

- Rust unit: session traffic warm-start emits `TrafficWarmStart`; ego move changes source; `continue_search` can emit multiple `Improved` updates on fake map.
- Demo API: WS or HTTP stream receives ≥1 intermediate update before final on long `run_for` budget split into slices.
- Existing oracle / compare_cpp tests remain green (no behavior change to core IMOMD math).

## Rollout

1. Rust `navigation` module + tests.
2. Python bindings for session / plan updates.
3. Demo backend uses session; WS emits `plan_update`.
4. Frontend anytime + ego + cost curve.
5. Document how to add a new plugin.
