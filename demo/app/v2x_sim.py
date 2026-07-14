"""V2X traffic event simulator for the demo."""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from typing import Literal

TrafficLabel = Literal["free", "slow", "jam", "blocked"]

LEVELS: list[TrafficLabel] = ["free", "slow", "jam", "blocked"]
WEIGHTS = [0.45, 0.30, 0.20, 0.05]


@dataclass
class V2xEvent:
    kind: Literal["zone", "clear"]
    nodes: list[int]
    level: TrafficLabel
    message: str


def pick_spread_destinations(edges: list[dict], node_count: int) -> tuple[int, list[int], int]:
    """Pick reachable source / one objective / target spread across the graph."""
    adj: dict[int, set[int]] = {i: set() for i in range(node_count)}
    for e in edges:
        if not isinstance(e.get("weight"), (int, float)) or e["weight"] == float("inf"):
            continue
        adj[e["from"]].add(e["to"])
        adj[e["to"]].add(e["from"])

    start = next((i for i in range(node_count) if adj[i]), 0)
    dist: dict[int, int] = {start: 0}
    queue: deque[int] = deque([start])
    while queue:
        u = queue.popleft()
        for v in adj[u]:
            if v not in dist:
                dist[v] = dist[u] + 1
                queue.append(v)

    reachable = sorted(dist.keys())
    if len(reachable) < 3:
        return 0, [1], max(2, node_count - 1)

    pick = lambda frac: reachable[min(len(reachable) - 1, round((len(reachable) - 1) * frac))]
    return start, [pick(0.33)], reachable[-1]


class V2xSimulator:
    """Generates random zone traffic updates mimicking V2X broadcasts."""

    def __init__(self, node_count: int, *, seed: int = 0) -> None:
        self.node_count = node_count
        self.rng = random.Random(seed)
        self.tick = 0

    def next_event(self) -> V2xEvent:
        self.tick += 1
        if self.tick % 6 == 0:
            return V2xEvent(
                kind="clear",
                nodes=[],
                level="free",
                message="V2X: city-wide traffic report — congestion easing",
            )

        zone_size = self.rng.randint(1, max(2, self.node_count // 20))
        nodes = self.rng.sample(range(self.node_count), k=min(zone_size, self.node_count))
        level = self.rng.choices(LEVELS, weights=WEIGHTS, k=1)[0]
        label = {
            "free": "free flow — speed limit restored",
            "slow": "slow traffic detected",
            "jam": "heavy congestion reported",
            "blocked": "road closure / accident",
        }[level]
        return V2xEvent(
            kind="zone",
            nodes=nodes,
            level=level,
            message=f"V2X zone {nodes}: {label}",
        )
