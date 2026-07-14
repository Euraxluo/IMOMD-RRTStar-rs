# Adding a Navigation Planner Plugin

The realtime demo is driven by `NavigationSession` + `PlannerPlugin`.
IMOMD-RRT* is the first plugin (`imomd`). Future dynamic-weight algorithms
(LPA*, D* Lite, CCH+A*, …) should implement the same Rust trait and appear in
the demo algorithm list.

## Contract

```rust
pub trait PlannerPlugin: Send + Sync {
    fn id(&self) -> &'static str;
    fn reset(&mut self, graph: Arc<AdjacencyGraph>, destinations: Destinations) -> Result<()>;
    fn on_graph_changed(&mut self, graph: Arc<AdjacencyGraph>) -> Result<GraphUpdateStats>;
    fn on_ego_moved(&mut self, ego_node: NodeId, remaining: Destinations) -> Result<()>;
    fn continue_search(&mut self, budget: Duration) -> Result<Vec<PlanUpdate>>;
    fn best(&self) -> Option<&PlanningResult>;
    fn is_finished(&self) -> bool;
}
```

`NavigationSession` owns graph revisions and translates domain events into these
calls. Clients (FastAPI / Python) must not branch on algorithm-specific APIs.

## Steps to add `lpa_star`

1. Create `src/navigation/lpa_plugin.rs` implementing `PlannerPlugin`.
2. Export it from `src/navigation/mod.rs`.
3. Register the id in `PyNavigationSession::new` (`src/python/mod.rs`).
4. Mark it available in `DemoState.snapshot()["algorithms"]`.
5. Add a focused test in `tests/navigation_session.rs`.

## Event semantics

| Event | Plugin call |
|---|---|
| destinations set | `reset` then `continue_search` |
| traffic / edge weights change | `on_graph_changed` then `continue_search` |
| ego / vehicle snapped node | `on_ego_moved` then `continue_search` |
| anytime tick | `continue_search` only |

Emit `PlanUpdate`s whenever the best path or cost changes so the UI can animate
improvement the way the paper GIF does.
