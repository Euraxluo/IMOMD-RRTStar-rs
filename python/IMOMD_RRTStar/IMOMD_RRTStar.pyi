from typing import List

class FakeMap:
    @staticmethod
    def load(map_type: int) -> "AdjacencyGraph": ...

class AdjacencyGraph:
    @property
    def node_count(self) -> int: ...

class PlanningResult:
    path: List[int]
    visit_order: List[int]
    cost: float
    explored_nodes: int
    elapsed_secs: float

class ImomdPlanner:
    def __init__(
        self,
        graph: AdjacencyGraph,
        source: int,
        objectives: List[int],
        target: int,
        max_iter: int = 1_000_000,
        max_time_secs: int = 60,
        goal_bias: float = 1.0,
    ) -> None: ...
    def run_for(self, seconds: float) -> PlanningResult: ...
    def tree_count(self) -> int: ...
