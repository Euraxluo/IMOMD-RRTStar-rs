"""High-level Python wrapper for IMOMD-RRTStar Rust core."""

from __future__ import annotations

from ._imomd_native import (
    AdjacencyGraph,
    AlgorithmConfig,
    CustomGraph,
    FakeMap,
    GraphUpdateStats,
    ImomdPlanner,
    OsmMap,
    PlanningResult,
    TrafficGraph,
)

__all__ = [
    "FakeMap",
    "AlgorithmConfig",
    "GraphUpdateStats",
    "CustomGraph",
    "TrafficGraph",
    "ImomdPlanner",
    "PlanningResult",
    "AdjacencyGraph",
    "OsmMap",
    "plan_fake_map",
]


def plan_fake_map(
    map_type: int = -1,
    source: int = 0,
    objectives: list[int] | None = None,
    target: int = 2,
    max_time_secs: float = 60.0,
    goal_bias: float = 1.0,
) -> PlanningResult:
    """Convenience entry point for planning on built-in test maps."""
    if objectives is None:
        objectives = [1]
    graph = FakeMap.load(map_type)
    planner = ImomdPlanner(
        graph,
        source,
        objectives,
        target,
        max_iter=1_000_000,
        max_time_secs=int(max_time_secs),
        goal_bias=goal_bias,
    )
    return planner.run_for(max_time_secs)
