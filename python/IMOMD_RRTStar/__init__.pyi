from typing import List, Literal, Optional, Tuple, TypedDict, overload

TrafficLevel = Literal[
    "free",
    "clear",
    "normal",
    "slow",
    "congested",
    "jam",
    "blocked_slow",
    "blocked",
    "closed",
]
StepStatus = Literal["expanded", "connected", "path_improved", "finished"]

class NodeView(TypedDict):
    id: int
    lat: float
    lon: float

EdgeView = TypedDict(
    "EdgeView",
    {
        "from": int,
        "to": int,
        "base_weight": float,
        "weight": float,
        "level": str,
    },
)

class TrafficView(TypedDict):
    nodes: List[NodeView]
    edges: List[EdgeView]

class PlannerStep(TypedDict):
    status: StepStatus
    iteration: int
    best_cost: Optional[float]

class AlgorithmConfig:
    @staticmethod
    def from_yaml(path: str) -> AlgorithmConfig: ...
    @staticmethod
    def from_yaml_string(yaml: str) -> AlgorithmConfig: ...
    def to_yaml_string(self) -> str: ...
    @property
    def source(self) -> int: ...
    @property
    def objectives(self) -> List[int]: ...
    @property
    def target(self) -> int: ...
    @property
    def max_iter(self) -> int: ...
    @property
    def max_time_secs(self) -> int: ...
    @property
    def goal_bias(self) -> float: ...

class AdjacencyGraph:
    @property
    def node_count(self) -> int: ...

class FakeMap:
    @staticmethod
    def load(map_type: int) -> AdjacencyGraph: ...

class CustomGraph:
    @staticmethod
    def load(path: str) -> AdjacencyGraph: ...

class OsmMap:
    @staticmethod
    def load(
        osm_path: str, *, osm_way_config: str = "config/osm_way_config.yaml"
    ) -> AdjacencyGraph: ...

class TrafficGraph:
    @staticmethod
    def from_edges(
        nodes: List[Tuple[float, float]], edges: List[Tuple[int, int, float]]
    ) -> TrafficGraph: ...
    @staticmethod
    def load_fake(map_type: int) -> TrafficGraph: ...
    @staticmethod
    def load_osm(
        osm_path: str, *, osm_way_config: str = "config/osm_way_config.yaml"
    ) -> TrafficGraph: ...
    @property
    def node_count(self) -> int: ...
    def set_edge_traffic(
        self, from_: int, to: int, level: TrafficLevel
    ) -> None: ...
    def set_zone_traffic(
        self, nodes: List[int], level: TrafficLevel
    ) -> None: ...
    def clear_traffic(self) -> None: ...
    def materialize(self) -> AdjacencyGraph: ...
    def export_view(self) -> TrafficView: ...

class PlanningResult:
    path: List[int]
    visit_order: List[int]
    cost: float
    explored_nodes: int
    elapsed_secs: float

class GraphUpdateStats:
    previous_tree_nodes: int
    retained_tree_nodes: int
    pruned_tree_nodes: int

class ImomdPlanner:
    @overload
    def __init__(
        self, graph: AdjacencyGraph, source_or_config: AlgorithmConfig
    ) -> None: ...
    @overload
    def __init__(
        self,
        graph: AdjacencyGraph,
        source_or_config: int,
        objectives: List[int],
        target: int,
        *,
        max_iter: int = 1_000_000,
        max_time_secs: int = 60,
        goal_bias: float = 1.0,
    ) -> None: ...
    def run_for(self, seconds: float) -> PlanningResult: ...
    def run_until(self, seconds: float) -> PlanningResult: ...
    def update_graph(self, graph: AdjacencyGraph) -> GraphUpdateStats: ...
    def step(self) -> PlannerStep: ...
    def best_result(self) -> Optional[PlanningResult]: ...
    def tree_count(self) -> int: ...
    @property
    def is_finished(self) -> bool: ...

class PlanUpdate(TypedDict, total=False):
    sequence: int
    reason: str
    path: Optional[List[int]]
    cost: Optional[float]
    visit_order: Optional[List[int]]
    explored_nodes: Optional[int]
    replan_mode: str
    ego_node: Optional[int]
    algorithm_id: str
    tree_update: Optional[dict]

class NavigationSession:
    def __init__(self, algorithm: str = "imomd") -> None: ...
    @property
    def algorithm_id(self) -> str: ...
    @property
    def ego_node(self) -> Optional[int]: ...
    def set_graph(self, graph: AdjacencyGraph) -> None: ...
    def snap_ego(self, latitude: float, longitude: float) -> int: ...
    def set_destinations(
        self,
        source: int,
        objectives: List[int],
        target: int,
        budget_secs: float = 0.5,
    ) -> List[PlanUpdate]: ...
    def on_traffic_changed(self, budget_secs: float = 0.5) -> List[PlanUpdate]: ...
    def on_ego_moved(self, ego_node: int, budget_secs: float = 0.5) -> List[PlanUpdate]: ...
    def continue_search(self, budget_secs: float = 0.3) -> List[PlanUpdate]: ...
    def best(self) -> Optional[PlanningResult]: ...

def plan_fake_map(
    map_type: int = -1,
    source: int = 0,
    objectives: Optional[List[int]] = None,
    target: int = 2,
    max_time_secs: float = 60.0,
    goal_bias: float = 1.0,
) -> PlanningResult: ...
