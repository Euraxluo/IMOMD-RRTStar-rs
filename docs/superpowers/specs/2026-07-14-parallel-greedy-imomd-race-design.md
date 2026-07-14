# Parallel Greedy + IMOMD + Exact Race — Design

**Date:** 2026-07-14  
**Status:** Approved for implementation  
**Context:** Many waypoints make pure IMOMD slow to first path. Speed is a product requirement: race greedy / IMOMD / exact solvers and stream whoever wins first. Exact lane provides theoretically optimal tours on the current graph (bounded waypoint count).

## Goals

1. **Fast first path** under many destinations without freezing the UI.
2. **Race semantics** — display whichever solver produces a feasible path first; do **not** prefer greedy (or any lane) by default.
3. **Anytime / exact cover** — later results replace the displayed path only when cost is strictly better.
4. **Exact optimality lane** — Dijkstra pairwise + exact TSP (enumerate) for small waypoint counts; theoretically optimal on the *current* graph.
5. **Parallelism** — lanes overlap in wall-clock time where safe.
6. **Keep IMOMD identity** — multi-tree anytime search remains the improving approximate lane.

## Non-goals (v1)

- Full D* Lite / LPA* (plugin stubs stay; exact Dijkstra+TSP covers “optimal on current graph”).
- Injecting greedy/exact paths into IMOMD tree state (warm-start trees).
- Exact TSP for very large waypoint counts (cap / skip).

## Race model

```
DestinationsSet / (re)start
    ├─ GreedyLane   (NN/CI order + leg shortest paths)
    ├─ ImomdLane    (ImomdPlugin continue_search / warm-start)
    └─ ExactLane    (all-pairs Dijkstra + exact TSP)  [if |obj| ≤ N]
            │
            ▼
     BestCostGate:
       - first feasible PlanUpdate wins the UI slot
       - subsequent updates apply only if cost < best - ε
```

### Display rules ( Normative )

1. **Who finishes first is shown first** — no default-to-greedy.
2. **Only a strictly lower cost may replace** the displayed path (`cost_new < best - ε`).
3. Each update carries `algorithm_id` / `reason` so the UI can label source (`greedy` / `imomd` / `exact`).

### Event handling ( Normative )

| Event | Behavior |
|---|---|
| **DestinationsSet** (途经/起终变了) | **Cancel all in-flight lanes**, clear best gate, **restart full race** (greedy + imomd + exact if eligible). |
| **TrafficChanged** (路况变了) | **Do not cold-reset IMOMD.** Exact (and greedy) **recompute on new weights**; IMOMD **`update_graph` warm-start** then continue. Invalidate displayed best if its cost is stale under new weights (re-publish when any lane returns a feasible path; gate resets so first new feasible can show even if numerically worse than obsolete best). |
| **EgoMoved** | Treat like destinations change for remaining goals: cancel race, restart with new source. |
| **ContinueSearch** | Feed **IMOMD only** (anytime). Exact/greedy are one-shot per race epoch unless traffic/destinations restart them. |

## Lanes

### Greedy

- Pairwise costs among terminals (parallel Dijkstra when `T` large; Haversine fallback only if needed for ordering).
- Visit order: nearest-neighbor or cheapest-insertion (fixed source/target). **Not** `n!` brute force.
- Materialize concatenated Dijkstra legs → `PlanUpdate(reason=GreedyInit)`.

### IMOMD

- Existing `ImomdPlugin`.
- Traffic: `on_graph_changed` / `update_graph` warm-start.
- Large `T`: reduce GA pressure so budget goes to tree growth.

### Exact (optimal on current graph)

- For each pair of terminals: Dijkstra on `AdjacencyGraph`.
- Visit order: exact enumeration of objective permutations when `|objectives| ≤ N` (default **N = 8**).
- If `|objectives| > N`: lane **skips** (emit nothing); race continues with greedy + IMOMD.
- Output is the exact shortest tour for the current weights under the visit-all-objectives model.
- `algorithm_id = "exact"`, `reason = ExactOptimal` (or similar).

## Session architecture

`NavigationSession` becomes a **lane orchestrator** + **BestCostGate**:

```rust
// Pseudocode
on DestinationsSet:
  cancel_epoch(); epoch += 1;
  gate.reset();
  spawn/run greedy → gate.admit(update)
  spawn/run exact (if eligible) → gate.admit(update)
  reset imomd; continue_search(budget) → gate.admit(updates)

on TrafficChanged:
  epoch += 1 for stale cancellation of old greedy/exact tasks
  gate.invalidate_for_traffic(); // allow first new feasible through
  greedy.recompute → gate.admit
  exact.recompute → gate.admit
  imomd.on_graph_changed; continue_search → gate.admit
```

**Implementation order**

1. Rust: graph Dijkstra helper + `ExactPlugin` + greedy init helper + `BestCostGate` in session.  
2. Parallel: prefer Rust threads/`rayon` for pairwise Dijkstra; session may run greedy+exact before/around first IMOMD slice, then stream IMOMD. True async cancel is best-effort via epoch id.  
3. Demo: consume `algorithm_id` / new reasons; no “force greedy first” UI.

Fallback if session-parallel is sticky: demo `asyncio.gather` with the **same gate rules** (documented equivalent).

## Testing

1. Few waypoints: either lane may win first; later exact (if any) must not be rejected when better.
2. Many waypoints (15–30): greedy usually first; exact skipped; no UI freeze.
3. Cost gate: worse second result does not replace best.
4. Destinations change: no stale path from previous OD.
5. Traffic change: IMOMD reports warm_start; exact/greedy recompute; display can update to new-graph feasible path.
6. Existing navigation_session tests still pass.

## Acceptance

- ~30 waypoints: a path appears quickly (race), page does not hang on `n!` oracle.
- First on-screen path = first finisher, not hard-coded greedy.
- With ≤8 objectives, exact lane can publish the graph-optimal tour and cover when better.
- Traffic uses IMOMD warm-start; destinations trigger full race restart.
