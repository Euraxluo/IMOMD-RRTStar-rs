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
    """Pick reachable source / objectives / target spread across the graph.

    Large maps use 2–3 objectives so IMOMD's multi-objective visit-order search
    has room to improve over time (paper-style anytime). Tiny maps keep one.
    """
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
    target = reachable[-1]
    if node_count >= 1500 and len(reachable) >= 8:
        objectives = [pick(0.25), pick(0.5), pick(0.75)]
    elif node_count >= 400 and len(reachable) >= 5:
        objectives = [pick(0.33), pick(0.66)]
    else:
        objectives = [pick(0.33)]
    # Drop accidental duplicates / collisions with source/target.
    cleaned: list[int] = []
    for obj in objectives:
        if obj in (start, target) or obj in cleaned:
            continue
        cleaned.append(obj)
    if not cleaned:
        cleaned = [pick(0.5)]
        if cleaned[0] in (start, target):
            cleaned = [reachable[len(reachable) // 2]]
    return start, cleaned, target


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

        zone_size = self.rng.randint(1, max(2, min(36, self.node_count // 20)))
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
