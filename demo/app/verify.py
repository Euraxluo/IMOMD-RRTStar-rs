"""Simple Dijkstra oracle to verify IMOMD planner paths."""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass
from typing import Any


@dataclass
class VerifyReport:
    ok: bool
    planner_cost: float | None
    recomputed_cost: float | None
    oracle_cost: float | None
    cost_delta: float | None
    oracle_gap: float | None
    path_valid: bool
    visits_objectives: bool
    starts_at_source: bool
    ends_at_target: bool
    broken_edges: list[tuple[int, int]]
    message: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "planner_cost": self.planner_cost,
            "recomputed_cost": self.recomputed_cost,
            "oracle_cost": self.oracle_cost,
            "cost_delta": self.cost_delta,
            "oracle_gap": self.oracle_gap,
            "path_valid": self.path_valid,
            "visits_objectives": self.visits_objectives,
            "starts_at_source": self.starts_at_source,
            "ends_at_target": self.ends_at_target,
            "broken_edges": self.broken_edges,
            "message": self.message,
        }


def build_graph(edges: list[dict[str, Any]]) -> dict[int, dict[int, float]]:
    """Adjacency list from demo export_view edges."""
    graph: dict[int, dict[int, float]] = {}
    for e in edges:
        w = e.get("weight", e.get("effective_weight"))
        if not isinstance(w, (int, float)) or not math.isfinite(w):
            continue
        u, v = int(e["from"]), int(e["to"])
        graph.setdefault(u, {})[v] = float(w)
        graph.setdefault(v, {})[u] = float(w)
    return graph


def dijkstra(graph: dict[int, dict[int, float]], source: int, target: int) -> tuple[float, list[int]]:
    """Classic Dijkstra — exact shortest path on current graph."""
    if source == target:
        return 0.0, [source]

    dist: dict[int, float] = {source: 0.0}
    parent: dict[int, int | None] = {source: None}
    heap: list[tuple[float, int]] = [(0.0, source)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, math.inf):
            continue
        if u == target:
            break
        for v, w in graph.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                parent[v] = u
                heapq.heappush(heap, (nd, v))

    if target not in dist or not math.isfinite(dist[target]):
        return math.inf, []

    path: list[int] = []
    cur: int | None = target
    while cur is not None:
        path.append(cur)
        cur = parent.get(cur)
    path.reverse()
    return dist[target], path


def path_cost(graph: dict[int, dict[int, float]], path: list[int]) -> tuple[float, list[tuple[int, int]]]:
    if len(path) < 2:
        return 0.0, []
    total = 0.0
    broken: list[tuple[int, int]] = []
    for a, b in zip(path, path[1:]):
        w = graph.get(a, {}).get(b)
        if w is None or not math.isfinite(w):
            broken.append((a, b))
            return math.inf, broken
        total += w
    return total, broken


def oracle_mo_cost(
    graph: dict[int, dict[int, float]],
    source: int,
    objectives: list[int],
    target: int,
    *,
    max_brute_force_objectives: int = 6,
) -> tuple[float | None, list[int]]:
    """Brute-force visit order + Dijkstra legs (exact for small objective count).

    Returns ``(None, [])`` when there are too many middle objectives — exact
    search is ``n!`` permutations and would freeze the demo (30! ≈ 2.65e32).
    """
    if not objectives:
        return dijkstra(graph, source, target)

    if len(objectives) > max_brute_force_objectives:
        return None, []

    best_cost = math.inf
    best_path: list[int] = []
    for perm in itertools.permutations(objectives):
        legs = [source, *perm, target]
        cost = 0.0
        full: list[int] = []
        ok = True
        for u, v in zip(legs, legs[1:]):
            leg_cost, leg_path = dijkstra(graph, u, v)
            if not math.isfinite(leg_cost) or not leg_path:
                ok = False
                break
            cost += leg_cost
            if not full:
                full.extend(leg_path)
            else:
                full.extend(leg_path[1:])
        if ok and cost < best_cost:
            best_cost = cost
            best_path = full
    return best_cost, best_path


def verify_plan(
    *,
    edges: list[dict[str, Any]],
    path: list[int] | None,
    planner_cost: float | None,
    source: int,
    objectives: list[int],
    target: int,
    cost_tol: float = 1e-3,
    oracle_tol: float = 0.05,
) -> VerifyReport:
    """Verify planner output against graph arithmetic and Dijkstra oracle."""
    graph = build_graph(edges)

    if not path:
        return VerifyReport(
            ok=False,
            planner_cost=planner_cost,
            recomputed_cost=None,
            oracle_cost=None,
            cost_delta=None,
            oracle_gap=None,
            path_valid=False,
            visits_objectives=False,
            starts_at_source=False,
            ends_at_target=False,
            broken_edges=[],
            message="no path to verify",
        )

    recomputed, broken = path_cost(graph, path)
    starts = path[0] == source
    ends = path[-1] == target
    visits = all(obj in path for obj in objectives)
    path_valid = not broken and math.isfinite(recomputed)

    oracle_cost, _ = oracle_mo_cost(graph, source, objectives, target)
    oracle_available = oracle_cost is not None and math.isfinite(oracle_cost)

    cost_delta = None
    if planner_cost is not None and math.isfinite(recomputed):
        cost_delta = abs(planner_cost - recomputed)

    oracle_gap = None
    if planner_cost is not None and oracle_available and math.isfinite(planner_cost):
        oracle_gap = planner_cost - float(oracle_cost)

    ok = (
        path_valid
        and starts
        and ends
        and visits
        and cost_delta is not None
        and cost_delta <= cost_tol
        and (not oracle_available or (oracle_gap is not None and oracle_gap >= -cost_tol))
    )

    if not oracle_available and ok:
        msg = (
            f"OK — path valid; exact Dijkstra oracle skipped "
            f"({len(objectives)} objectives > 6, would be n!)"
        )
    elif ok and oracle_gap is not None and oracle_cost and oracle_gap > oracle_cost * oracle_tol:
        msg = (
            f"OK (anytime) — path valid; planner {planner_cost:.1f}m vs "
            f"Dijkstra oracle {oracle_cost:.1f}m (+{oracle_gap:.1f}m)"
        )
    elif ok:
        msg = f"OK — path valid, cost matches graph; oracle {oracle_cost:.1f}m"
    elif not path_valid:
        msg = f"path uses missing/blocked edges: {broken[:5]}"
    elif not visits:
        msg = "path does not visit all objectives"
    elif cost_delta is not None and cost_delta > cost_tol:
        msg = f"reported cost {planner_cost:.2f} != recomputed {recomputed:.2f}"
    elif (
        oracle_gap is not None
        and oracle_cost is not None
        and oracle_gap > oracle_cost * oracle_tol + cost_tol
    ):
        msg = f"planner cost {planner_cost:.2f} exceeds oracle {oracle_cost:.2f} by {oracle_gap:.2f}m"
    else:
        msg = "verification failed"

    return VerifyReport(
        ok=ok,
        planner_cost=planner_cost,
        recomputed_cost=recomputed if math.isfinite(recomputed) else None,
        oracle_cost=float(oracle_cost) if oracle_available else None,
        cost_delta=cost_delta,
        oracle_gap=oracle_gap,
        path_valid=path_valid,
        visits_objectives=visits,
        starts_at_source=starts,
        ends_at_target=ends,
        broken_edges=broken,
        message=msg,
    )
