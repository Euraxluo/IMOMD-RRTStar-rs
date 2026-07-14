"""FastAPI backend: IMOMD-RRT* + V2X dynamic traffic demo."""

from __future__ import annotations

import asyncio
import os
import threading
from contextlib import asynccontextmanager
from contextlib import suppress
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, TypeVar

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from IMOMD_RRTStar import NavigationSession, TrafficGraph

from .v2x_sim import V2xSimulator, pick_spread_destinations
from .verify import verify_plan

ROOT = Path(__file__).resolve().parents[2]
STATIC = Path(__file__).resolve().parents[1] / "static"
DEFAULT_OSM = ROOT / "tmp/imomd-cpp/osm_data/FRB2.osm"


class DestinationsBody(BaseModel):
    source: int = 0
    objectives: list[int] = Field(default_factory=lambda: [1])
    target: int = 2


class EdgeTrafficBody(BaseModel):
    from_node: int = Field(alias="from")
    to: int
    level: str

    model_config = {"populate_by_name": True}


class EdgeRef(BaseModel):
    from_node: int = Field(alias="from")
    to: int

    model_config = {"populate_by_name": True}


class EdgeListTrafficBody(BaseModel):
    edges: list[EdgeRef]
    level: str


class ZoneTrafficBody(BaseModel):
    nodes: list[int]
    level: str


class MapSwitchBody(BaseModel):
    map_key: str


class EgoBody(BaseModel):
    node: int | None = None
    lat: float | None = None
    lon: float | None = None


def _updates_to_result(updates: list[dict[str, Any]], session: NavigationSession) -> dict[str, Any]:
    best = session.best()
    chosen: dict[str, Any] | None = None
    for update in reversed(updates):
        if update.get("path"):
            chosen = update
            break
    if chosen is None and best is not None:
        return {
            "path": list(best.path),
            "cost": float(best.cost),
            "explored_nodes": int(best.explored_nodes),
            "elapsed_secs": float(best.elapsed_secs),
            "visit_order": list(best.visit_order),
            "replan_mode": "resume",
            "tree_update": None,
            "updates": updates,
        }
    if chosen is None:
        raise ValueError("no feasible route under current conditions")
    tree = chosen.get("tree_update")
    return {
        "path": chosen["path"],
        "cost": chosen["cost"],
        "explored_nodes": chosen.get("explored_nodes") or 0,
        "elapsed_secs": 0.0,
        "visit_order": chosen.get("visit_order") or [],
        "replan_mode": chosen.get("replan_mode") or "resume",
        "tree_update": tree,
        "updates": updates,
        "ego_node": chosen.get("ego_node"),
        "algorithm_id": chosen.get("algorithm_id") or session.algorithm_id,
    }


@dataclass
class DemoState:
    traffic: TrafficGraph
    map_name: str
    map_key: str
    destinations: DestinationsBody = field(default_factory=DestinationsBody)
    session: NavigationSession = field(default_factory=lambda: NavigationSession("imomd"))
    traffic_revision: int = 0
    last_replan_mode: str = "fresh"
    last_update_stats: dict[str, int] | None = None
    last_result: dict[str, Any] | None = None
    cost_history: list[dict[str, Any]] = field(default_factory=list)
    v2x: V2xSimulator | None = None
    auto_v2x: bool = False
    auto_anytime: bool = True
    event_log: list[str] = field(default_factory=list)
    ego_node: int | None = None
    pending_traffic: bool = False
    seeded: bool = False

    def log(self, msg: str) -> None:
        self.event_log.append(msg)
        self.event_log = self.event_log[-30:]

    def traffic_changed(self) -> None:
        self.traffic_revision += 1
        self.pending_traffic = True

    def _sync_graph(self) -> None:
        self.session.set_graph(self.traffic.materialize())

    def _publish(self, updates: list[dict[str, Any]]) -> dict[str, Any]:
        payload = _updates_to_result(updates, self.session)
        self.last_replan_mode = payload["replan_mode"]
        self.last_update_stats = payload.get("tree_update")
        self.ego_node = payload.get("ego_node", self.session.ego_node)
        self.last_result = payload
        self.seeded = True
        for update in updates:
            cost = update.get("cost")
            if cost is None:
                continue
            self.cost_history.append(
                {
                    "sequence": update.get("sequence"),
                    "cost": cost,
                    "reason": update.get("reason"),
                }
            )
        self.cost_history = self.cost_history[-80:]
        return payload

    def replan(self, seconds: float = 1.0) -> dict[str, Any]:
        d = self.destinations
        # Only push a new graph snapshot when seeding or traffic actually changed.
        # Calling set_graph on every continue_search bumps graph_revision and
        # forces warm-start, which destroys anytime tree accumulation.
        if not self.seeded:
            self._sync_graph()
            updates = self.session.set_destinations(
                d.source, d.objectives, d.target, budget_secs=seconds
            )
        elif self.pending_traffic:
            self._sync_graph()
            updates = self.session.on_traffic_changed(budget_secs=seconds)
            self.pending_traffic = False
        else:
            updates = self.session.continue_search(budget_secs=seconds)
        return self._publish(updates)

    def anytime_budget(self) -> float:
        """Larger maps need longer slices or the UI never sees cost drops."""
        nodes = int(self.traffic.node_count)
        if nodes >= 5000:
            return 1.2
        if nodes >= 1500:
            return 0.8
        return 0.35

    def replace_destinations(
        self, destinations: DestinationsBody, seconds: float = 1.5
    ) -> dict[str, Any]:
        self.destinations = destinations
        self.pending_traffic = False
        self._sync_graph()
        updates = self.session.set_destinations(
            destinations.source,
            destinations.objectives,
            destinations.target,
            budget_secs=seconds,
        )
        return self._publish(updates)

    def apply_traffic_and_replan(self, seconds: float = 1.0) -> dict[str, Any]:
        self._sync_graph()
        updates = self.session.on_traffic_changed(budget_secs=seconds)
        self.pending_traffic = False
        return self._publish(updates)

    def set_ego(self, node: int, seconds: float = 1.0) -> dict[str, Any]:
        self.ego_node = node
        objectives = [o for o in self.destinations.objectives if o != node]
        self.destinations = DestinationsBody(
            source=node,
            objectives=objectives,
            target=self.destinations.target,
        )
        updates = self.session.on_ego_moved(node, budget_secs=seconds)
        return self._publish(updates)

    def continue_anytime(self, seconds: float = 0.25) -> dict[str, Any] | None:
        if not self.seeded:
            return None
        updates = self.session.continue_search(budget_secs=seconds)
        if not any(u.get("path") for u in updates) and not any(
            u.get("reason") in {"improved", "connected", "finished"} for u in updates
        ):
            # Still publish resume snapshots so UI can animate cost plateaus.
            if not updates:
                return None
        return self._publish(updates)
    def snapshot(self) -> dict[str, Any]:
        view = self.traffic.export_view()
        path = (self.last_result or {}).get("path")
        cost = (self.last_result or {}).get("cost")
        d = self.destinations
        verification = verify_plan(
            edges=view["edges"],
            path=path,
            planner_cost=cost,
            source=d.source,
            objectives=d.objectives,
            target=d.target,
        ).to_dict()
        return {
            "available_maps": available_maps(),
            "map_key": self.map_key,
            "map_name": self.map_name,
            "node_count": self.traffic.node_count,
            "destinations": d.model_dump(),
            "view": view,
            "path": path,
            "cost": cost,
            "visit_order": (self.last_result or {}).get("visit_order"),
            "verification": verification,
            "events": self.event_log,
            "auto_v2x": self.auto_v2x,
            "auto_anytime": self.auto_anytime,
            "v2x_tick": self.v2x.tick if self.v2x is not None else 0,
            "replan_mode": self.last_replan_mode,
            "tree_update": self.last_update_stats,
            "traffic_revision": self.traffic_revision,
            "ego_node": self.ego_node if self.ego_node is not None else d.source,
            # Avoid touching NavigationSession here — PyO3 forbids concurrent
            # borrows while continue_search mutates the planner on another thread.
            "algorithm_id": (self.last_result or {}).get("algorithm_id") or "imomd",
            "cost_history": self.cost_history,
            "algorithms": [
                {
                    "id": "race",
                    "name": "赛跑：贪心 + IMOMD + Exact",
                    "available": True,
                },
                {"id": "imomd", "name": "IMOMD-RRT*（赛跑车道）", "available": True},
                {"id": "greedy", "name": "Greedy（赛跑车道）", "available": True},
                {
                    "id": "exact",
                    "name": "Exact Dijkstra+TSP（≤8 途经）",
                    "available": True,
                },
                {"id": "lpa_star", "name": "LPA*", "available": False},
                {"id": "d_star_lite", "name": "D* Lite", "available": False},
            ],
        }


def available_maps() -> list[dict[str, str]]:
    maps = [
        {
            "key": "chicago_mega",
            "name": "Chicago Mega Grid 80×64",
            "description": "芝加哥风格超大正交路网（含河岸桥梁与快速路），约 5000+ 节点",
        },
        {
            "key": "city_large",
            "name": "Synthetic City 24×18",
            "description": "非 OSM 中等网格/快速路任务，适合快速观察绕行",
        },
        {
            "key": "osm_or_fake",
            "name": "OSM FRB2 / fake fallback",
            "description": "优先加载本地 OSM；缺失时回退到 bugtrap fake map",
        },
        {
            "key": "bugtrap",
            "name": "Fake bugtrap",
            "description": "小型经典陷阱图，便于做算法回归检查",
        },
    ]
    chicago_osm = ROOT / "tmp" / "osm_data" / "chicago_downtown.osm"
    if chicago_osm.exists():
        maps.insert(
            1,
            {
                "key": "chicago_osm",
                "name": "Chicago Downtown OSM",
                "description": "真实 OSM 芝加哥市区路网（本地 tmp/osm_data/chicago_downtown.osm）",
            },
        )
    return maps


def _add_edge(edges: list[tuple[int, int, float]], a: int, b: int, weight: float) -> None:
    edges.append((a, b, weight))


def _load_city_map() -> tuple[TrafficGraph, str]:
    rows = 18
    cols = 24
    nodes: list[tuple[float, float]] = []
    edges: list[tuple[int, int, float]] = []

    def node_id(row: int, col: int) -> int:
        return row * cols + col

    for row in range(rows):
        for col in range(cols):
            nodes.append((float(row), float(col)))

    for row in range(rows):
        for col in range(cols):
            here = node_id(row, col)
            if col + 1 < cols:
                weight = 80.0
                if row in (4, 9, 14):
                    weight = 55.0
                _add_edge(edges, here, node_id(row, col + 1), weight)
            if row + 1 < rows:
                weight = 80.0
                if col in (5, 12, 19):
                    weight = 55.0
                _add_edge(edges, here, node_id(row + 1, col), weight)
            if row + 1 < rows and col + 1 < cols and row % 4 == 1 and col % 5 == 2:
                _add_edge(edges, here, node_id(row + 1, col + 1), 95.0)

    # A partial wall with three gates makes V2X jams visibly force detours.
    wall_col = 11
    gates = {3, 9, 15}
    edges = [
        edge
        for edge in edges
        if not (
            {edge[0] % cols, edge[1] % cols} == {wall_col, wall_col + 1}
            and edge[0] // cols not in gates
        )
    ]

    return TrafficGraph.from_edges(nodes, edges), "synthetic_city_24x18"


def _load_chicago_mega_map() -> tuple[TrafficGraph, str]:
    """Chicago-inspired mega orthogonal grid for large-scale anytime demos.

    Layout cues (not a cadastral survey):
    - dense rectangular street grid
    - Chicago River corridor with sparse bridges
    - Lake Michigan blank band on the east
    - expressway corridors with lower travel cost
    Coordinates are placed near real Chicago lat/lon for map feel.
    """
    rows = 64
    cols = 80
    lat0, lon0 = 41.78, -87.90
    dlat, dlon = 0.0032, 0.0040
    nodes: list[tuple[float, float]] = []
    edges: list[tuple[int, int, float]] = []

    def node_id(row: int, col: int) -> int:
        return row * cols + col

    # Lake Michigan: drop eastern shoreline columns from the graph entirely by
    # leaving them unconnected later; still allocate nodes so ids stay dense.
    lake_col = cols - 6
    river_col = 34
    river_bridges = {8, 16, 24, 32, 40, 48, 56}
    ew_express = {12, 28, 44, 56}
    ns_express = {10, 25, 45, 62}

    for row in range(rows):
        for col in range(cols):
            # Slight downtown density warp around the Loop.
            lat = lat0 + row * dlat
            lon = lon0 + col * dlon
            nodes.append((lat, lon))

    for row in range(rows):
        for col in range(cols):
            if col >= lake_col:
                continue
            here = node_id(row, col)
            if col + 1 < lake_col:
                # River removes most east-west links except bridges.
                crosses_river = col < river_col <= col + 1
                if crosses_river and row not in river_bridges:
                    pass
                else:
                    weight = 95.0
                    if row in ew_express:
                        weight = 48.0
                    elif crosses_river:
                        weight = 110.0  # bridge toll / delay
                    _add_edge(edges, here, node_id(row, col + 1), weight)
            if row + 1 < rows:
                weight = 95.0
                if col in ns_express:
                    weight = 48.0
                _add_edge(edges, here, node_id(row + 1, col), weight)
            # Occasional diagonal alley shortcuts downtown.
            if (
                20 <= row <= 44
                and 28 <= col <= 50
                and row + 1 < rows
                and col + 1 < lake_col
                and (row + col) % 7 == 0
            ):
                _add_edge(edges, here, node_id(row + 1, col + 1), 120.0)

    return (
        TrafficGraph.from_edges(nodes, edges),
        f"chicago_mega_{cols}x{rows}",
    )


def _load_traffic_map(map_key: str | None = None) -> tuple[TrafficGraph, str, str]:
    key = map_key or os.environ.get("DEMO_MAP", "city_large")
    if key == "chicago_mega":
        traffic, name = _load_chicago_mega_map()
        return traffic, name, key
    if key == "chicago_osm":
        path = ROOT / "tmp" / "osm_data" / "chicago_downtown.osm"
        if not path.exists():
            raise ValueError(
                "chicago_downtown.osm missing — run scripts/download_chicago_osm.py"
            )
        return TrafficGraph.load_osm(str(path)), path.name, key
    if key == "city_large":
        traffic, name = _load_city_map()
        return traffic, name, key
    if key == "bugtrap":
        return TrafficGraph.load_fake(-2), "fake_map_2 (bugtrap)", key
    if key != "osm_or_fake":
        raise ValueError(f"unknown map key: {key}")
    osm_path = os.environ.get("DEMO_OSM_PATH", str(DEFAULT_OSM))
    if Path(osm_path).exists():
        return TrafficGraph.load_osm(osm_path), Path(osm_path).name, key
    return TrafficGraph.load_fake(-2), "fake_map_2 (bugtrap)", key


def _initialize_state(map_key: str | None = None) -> DemoState:
    traffic, name, key = _load_traffic_map(map_key)
    demo = DemoState(traffic=traffic, map_name=name, map_key=key)
    demo.v2x = V2xSimulator(demo.traffic.node_count)
    view = traffic.export_view()
    src, objs, tgt = pick_spread_destinations(view["edges"], demo.traffic.node_count)
    demo.destinations = DestinationsBody(source=src, objectives=objs, target=tgt)
    demo.log(f"Loaded map: {name} ({demo.traffic.node_count} nodes)")
    return demo


state: DemoState
state_lock = asyncio.Lock()
planner_lock = threading.Lock()
clients: set[WebSocket] = set()

T = TypeVar("T")


def _planner_call(fn: Callable[[], T]) -> T:
    """Serialize planner mutations across asyncio.to_thread workers."""
    with planner_lock:
        return fn()


async def _broadcast(payload: dict[str, Any]) -> None:
    targets = tuple(clients)
    if not targets:
        return
    results = await asyncio.gather(
        *(client.send_json(payload) for client in targets),
        return_exceptions=True,
    )
    for client, result in zip(targets, results):
        if isinstance(result, Exception):
            clients.discard(client)


async def _v2x_loop() -> None:
    """Generate each V2X tick once, then broadcast one state to every client."""
    while True:
        await asyncio.sleep(3.0)
        async with state_lock:
            run_v2x = state.auto_v2x and state.v2x is not None
            if run_v2x:
                event = state.v2x.next_event()
                if event.kind == "clear":
                    state.traffic.clear_traffic()
                else:
                    state.traffic.set_zone_traffic(event.nodes, event.level)
                state.traffic_changed()
                state.log(event.message)
                event_message = event.message
            else:
                payload = {"type": "heartbeat", "state": state.snapshot()}
        if not run_v2x:
            await _broadcast(payload)
            continue
        try:
            result = await asyncio.to_thread(_planner_call, lambda: state.replan(0.8))
            async with state_lock:
                payload = {
                    "type": "update",
                    "event": event_message,
                    "state": state.snapshot(),
                    "result": result,
                }
        except ValueError as exc:
            async with state_lock:
                state.log(f"V2X: 当前路况下无可用路线: {exc}")
                payload = {
                    "type": "no_route",
                    "event": event_message,
                    "state": state.snapshot(),
                    "error": str(exc),
                }
        await _broadcast(payload)


async def _anytime_loop() -> None:
    """Continuously improve the active plan and stream updates to clients."""
    while True:
        await asyncio.sleep(0.2)
        async with state_lock:
            if not state.auto_anytime or state.auto_v2x or not state.seeded:
                continue
            budget = state.anytime_budget()
        try:
            result = await asyncio.to_thread(
                _planner_call, lambda b=budget: state.continue_anytime(b)
            )
        except ValueError:
            continue
        if result is None:
            continue
        async with state_lock:
            # Lightweight stream: path/cost only; avoid rebuilding full OSM view
            # on every slice (snapshot is still available via /api/state).
            payload = {
                "type": "plan_update",
                "state": {
                    "cost_history": state.cost_history[-40:],
                    "auto_anytime": state.auto_anytime,
                    "event_log": state.event_log[-8:],
                    "node_count": state.traffic.node_count,
                    "map_name": state.map_name,
                    "replan_mode": state.last_replan_mode,
                    "algorithm_id": (state.last_result or {}).get("algorithm_id"),
                },
                "result": result,
            }
        await _broadcast(payload)


@asynccontextmanager
async def lifespan(_: FastAPI):
    global state
    state = _initialize_state()
    try:
        # Short first slice on purpose: first path should be suboptimal so the
        # anytime loop can show continuous improvement (paper-style GIF).
        # On multi-objective Chicago OSM, ~50ms leaves a clearly worse path that
        # the next 0.8s slices typically cut by hundreds of meters.
        initial_budget = 0.05 if state.traffic.node_count >= 1500 else 0.2
        result = await asyncio.to_thread(
            _planner_call, lambda: state.replan(initial_budget)
        )
        cost = result.get("cost")
        if cost is not None:
            state.log(f"Initial plan cost={cost:.1f}m (anytime will keep improving)")
        else:
            state.log("Initial plan: searching…")
    except ValueError as exc:
        state.log(f"Initial plan skipped: {exc}")
    simulation_task = asyncio.create_task(_v2x_loop(), name="imomd-v2x-simulator")
    anytime_task = asyncio.create_task(_anytime_loop(), name="imomd-anytime")
    try:
        yield
    finally:
        for task in (simulation_task, anytime_task):
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
        clients.clear()


app = FastAPI(title="IMOMD-RRT* V2X Demo", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC / "index.html")


@app.get("/api/verify")
async def verify_state() -> dict[str, Any]:
    """Verify current path with Dijkstra oracle (simple exact algorithm)."""
    async with state_lock:
        snap = state.snapshot()
        report = snap["verification"]
        state.log(f"Verify: {report['message']}")
        return report


@app.get("/api/state")
async def get_state() -> dict[str, Any]:
    async with state_lock:
        return state.snapshot()


@app.get("/api/maps")
async def get_maps() -> dict[str, Any]:
    async with state_lock:
        return {"current": state.map_key, "maps": available_maps()}


@app.post("/api/map")
async def switch_map(body: MapSwitchBody) -> dict[str, Any]:
    global state
    async with state_lock:
        try:
            next_state = _initialize_state(body.map_key)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        next_state.auto_v2x = False
        budget = 0.05 if body.map_key in {"chicago_mega", "chicago_osm"} else 0.2
    try:
        result = await asyncio.to_thread(
            _planner_call, lambda: next_state.replan(budget)
        )
    except ValueError as exc:
        next_state.log(f"Initial plan skipped: {exc}")
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    async with state_lock:
        state = next_state
        return {"state": state.snapshot(), "result": result}


@app.post("/api/destinations")
async def set_destinations(body: DestinationsBody) -> dict[str, Any]:
    async with state_lock:
        state.log(f"Destinations updated: {body.model_dump()}")
    try:
        # Short budget so anytime can keep improving visibly after the first path.
        return await asyncio.to_thread(
            _planner_call, lambda: state.replace_destinations(body, 0.15)
        )
    except ValueError as exc:
        async with state_lock:
            state.log(f"规划失败: {exc}")
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/destinations/auto")
async def auto_destinations() -> dict[str, Any]:
    """Pick spread source / waypoint / target automatically."""
    async with state_lock:
        view = state.traffic.export_view()
        src, objs, tgt = pick_spread_destinations(view["edges"], state.traffic.node_count)
        destinations = DestinationsBody(source=src, objectives=objs, target=tgt)
        state.log(f"智能推荐路线: {src} → {objs} → {tgt}")
    try:
        result = await asyncio.to_thread(
            _planner_call, lambda: state.replace_destinations(destinations, 0.15)
        )
        return {"destinations": destinations.model_dump(), **result}
    except ValueError as exc:
        async with state_lock:
            state.log(f"规划失败: {exc}")
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/traffic/edge")
async def set_edge_traffic(body: EdgeTrafficBody) -> dict[str, Any]:
    async with state_lock:
        try:
            state.traffic.set_edge_traffic(body.from_node, body.to, body.level)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        state.traffic_changed()
        state.log(f"Edge {body.from_node}-{body.to} → {body.level}")
    try:
        return await asyncio.to_thread(_planner_call, lambda: state.replan(1.0))
    except ValueError as exc:
        async with state_lock:
            state.log(f"当前路况下无可用路线: {exc}")
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/traffic/edges")
async def set_edges_traffic(body: EdgeListTrafficBody) -> dict[str, Any]:
    if not body.edges:
        raise HTTPException(status_code=400, detail="edges must not be empty")
    if len(body.edges) > 2_000:
        raise HTTPException(status_code=422, detail="too many edges selected")
    async with state_lock:
        try:
            for edge in body.edges:
                state.traffic.set_edge_traffic(edge.from_node, edge.to, body.level)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        state.traffic_changed()
        state.log(f"{len(body.edges)} selected lanes → {body.level}")
    try:
        return await asyncio.to_thread(_planner_call, lambda: state.replan(1.0))
    except ValueError as exc:
        async with state_lock:
            state.log(f"当前路况下无可用路线: {exc}")
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/traffic/zone")
async def set_zone_traffic(body: ZoneTrafficBody) -> dict[str, Any]:
    async with state_lock:
        try:
            state.traffic.set_zone_traffic(body.nodes, body.level)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        state.traffic_changed()
        state.log(f"Zone {body.nodes} → {body.level}")
    try:
        return await asyncio.to_thread(_planner_call, lambda: state.replan(1.0))
    except ValueError as exc:
        async with state_lock:
            state.log(f"当前路况下无可用路线: {exc}")
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/traffic/clear")
async def clear_traffic() -> dict[str, Any]:
    async with state_lock:
        state.traffic.clear_traffic()
        state.traffic_changed()
        state.log("Traffic cleared")
    try:
        return await asyncio.to_thread(_planner_call, lambda: state.replan(1.0))
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/replan")
async def replan(seconds: float = 1.5) -> dict[str, Any]:
    if not 0.0 < seconds <= 30.0:
        raise HTTPException(status_code=422, detail="seconds must be within (0, 30]")
    async with state_lock:
        state.log("Manual replan triggered")
    try:
        return await asyncio.to_thread(
            _planner_call, lambda: state.replan(seconds)
        )
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/v2x/auto")
async def toggle_auto(enabled: bool = True) -> dict[str, str]:
    async with state_lock:
        state.auto_v2x = enabled
        state.log(f"Auto V2X {'ON' if enabled else 'OFF'}")
        return {"auto_v2x": str(enabled)}


@app.post("/api/anytime")
async def toggle_anytime(enabled: bool = True) -> dict[str, str]:
    async with state_lock:
        state.auto_anytime = enabled
        state.log(f"Anytime improve {'ON' if enabled else 'OFF'}")
        return {"auto_anytime": str(enabled)}


@app.post("/api/ego")
async def set_ego(body: EgoBody) -> dict[str, Any]:
    async with state_lock:
        try:
            if body.node is not None:
                node = body.node
            elif body.lat is not None and body.lon is not None:
                state._sync_graph()
                node = state.session.snap_ego(body.lat, body.lon)
            else:
                raise HTTPException(status_code=400, detail="provide node or lat/lon")
            state.log(f"Ego moved → node {node}")
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    try:
        return await asyncio.to_thread(
            _planner_call, lambda: state.set_ego(node, 1.0)
        )
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.websocket("/ws/v2x")
async def v2x_stream(ws: WebSocket) -> None:
    await ws.accept()
    async with state_lock:
        snapshot = state.snapshot()
    await ws.send_json({"type": "snapshot", "state": snapshot})
    clients.add(ws)
    try:
        while True:
            # The browser is receive-only; waiting here detects disconnects.
            # A single lifespan task owns simulation and broadcasts updates.
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)
