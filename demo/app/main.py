"""FastAPI backend: IMOMD-RRT* + V2X dynamic traffic demo."""

from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager
from contextlib import suppress
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from IMOMD_RRTStar import ImomdPlanner, TrafficGraph

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


@dataclass
class DemoState:
    traffic: TrafficGraph
    map_name: str
    map_key: str
    destinations: DestinationsBody = field(default_factory=DestinationsBody)
    planner: ImomdPlanner | None = None
    planner_destinations: tuple[int, tuple[int, ...], int] | None = None
    traffic_revision: int = 0
    planner_revision: int = -1
    last_replan_mode: str = "fresh"
    last_update_stats: dict[str, int] | None = None
    last_result: dict[str, Any] | None = None
    v2x: V2xSimulator | None = None
    auto_v2x: bool = False
    event_log: list[str] = field(default_factory=list)

    def log(self, msg: str) -> None:
        self.event_log.append(msg)
        self.event_log = self.event_log[-30:]

    def traffic_changed(self) -> None:
        self.traffic_revision += 1

    def replan(self, seconds: float = 1.0) -> dict[str, Any]:
        graph = self.traffic.materialize()
        d = self.destinations
        destination_key = (d.source, tuple(d.objectives), d.target)
        self.last_update_stats = None
        if self.planner is None or self.planner_destinations != destination_key:
            self.planner = ImomdPlanner(
                graph,
                d.source,
                d.objectives,
                d.target,
                max_iter=200_000,
                max_time_secs=30,
                goal_bias=1.0,
            )
            self.planner_destinations = destination_key
            self.planner_revision = self.traffic_revision
            self.last_replan_mode = "fresh"
        elif self.planner_revision != self.traffic_revision:
            stats = self.planner.update_graph(graph)
            self.planner_revision = self.traffic_revision
            self.last_replan_mode = "warm_start"
            self.last_update_stats = {
                "previous_tree_nodes": stats.previous_tree_nodes,
                "retained_tree_nodes": stats.retained_tree_nodes,
                "pruned_tree_nodes": stats.pruned_tree_nodes,
            }
        else:
            self.last_replan_mode = "resume"

        try:
            result = self.planner.run_for(seconds)
        except ValueError:
            # Never keep presenting an old route against a newly changed graph.
            self.last_result = None
            raise
        payload = {
            "path": result.path,
            "cost": result.cost,
            "explored_nodes": result.explored_nodes,
            "elapsed_secs": result.elapsed_secs,
            "visit_order": result.visit_order,
            "replan_mode": self.last_replan_mode,
            "tree_update": self.last_update_stats,
        }
        self.last_result = payload
        return payload

    def replace_destinations(
        self, destinations: DestinationsBody, seconds: float = 1.5
    ) -> dict[str, Any]:
        """Plan a new route transactionally, then publish it on success."""
        graph = self.traffic.materialize()
        planner = ImomdPlanner(
            graph,
            destinations.source,
            destinations.objectives,
            destinations.target,
            max_iter=200_000,
            max_time_secs=30,
            goal_bias=1.0,
        )
        result = planner.run_for(seconds)
        payload = {
            "path": result.path,
            "cost": result.cost,
            "explored_nodes": result.explored_nodes,
            "elapsed_secs": result.elapsed_secs,
            "visit_order": result.visit_order,
            "replan_mode": "fresh",
            "tree_update": None,
        }
        self.destinations = destinations
        self.planner = planner
        self.planner_destinations = (
            destinations.source,
            tuple(destinations.objectives),
            destinations.target,
        )
        self.planner_revision = self.traffic_revision
        self.last_replan_mode = "fresh"
        self.last_update_stats = None
        self.last_result = payload
        return payload

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
            "verification": verification,
            "events": self.event_log,
            "auto_v2x": self.auto_v2x,
            "v2x_tick": self.v2x.tick if self.v2x is not None else 0,
            "replan_mode": self.last_replan_mode,
            "tree_update": self.last_update_stats,
            "traffic_revision": self.traffic_revision,
        }


def available_maps() -> list[dict[str, str]]:
    return [
        {
            "key": "city_large",
            "name": "Synthetic City 24x18",
            "description": "非 OSM 大规模网格/快速路任务，适合观察绕行与重规划",
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


def _load_traffic_map(map_key: str | None = None) -> tuple[TrafficGraph, str, str]:
    key = map_key or os.environ.get("DEMO_MAP", "city_large")
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
clients: set[WebSocket] = set()


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
            if state.auto_v2x and state.v2x is not None:
                event = state.v2x.next_event()
                if event.kind == "clear":
                    state.traffic.clear_traffic()
                else:
                    state.traffic.set_zone_traffic(event.nodes, event.level)
                state.traffic_changed()
                state.log(event.message)
                try:
                    result = await asyncio.to_thread(state.replan, 0.8)
                    payload = {
                        "type": "update",
                        "event": event.message,
                        "state": state.snapshot(),
                        "result": result,
                    }
                except ValueError as exc:
                    state.log(f"V2X: 当前路况下无可用路线: {exc}")
                    payload = {
                        "type": "no_route",
                        "event": event.message,
                        "state": state.snapshot(),
                        "error": str(exc),
                    }
            else:
                payload = {"type": "heartbeat", "state": state.snapshot()}
        await _broadcast(payload)


@asynccontextmanager
async def lifespan(_: FastAPI):
    global state
    state = _initialize_state()
    try:
        result = await asyncio.to_thread(state.replan, 1.5)
        state.log(f"Initial plan cost={result['cost']:.1f}m")
    except ValueError as exc:
        state.log(f"Initial plan skipped: {exc}")
    simulation_task = asyncio.create_task(_v2x_loop(), name="imomd-v2x-simulator")
    try:
        yield
    finally:
        simulation_task.cancel()
        with suppress(asyncio.CancelledError):
            await simulation_task
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
        try:
            result = await asyncio.to_thread(next_state.replan, 1.5)
        except ValueError as exc:
            next_state.log(f"Initial plan skipped: {exc}")
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        state = next_state
        return {"state": state.snapshot(), "result": result}


@app.post("/api/destinations")
async def set_destinations(body: DestinationsBody) -> dict[str, Any]:
    async with state_lock:
        state.log(f"Destinations updated: {body.model_dump()}")
        try:
            return await asyncio.to_thread(state.replace_destinations, body, 1.5)
        except ValueError as exc:
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
            result = await asyncio.to_thread(state.replace_destinations, destinations, 1.5)
            return {"destinations": destinations.model_dump(), **result}
        except ValueError as exc:
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
            return await asyncio.to_thread(state.replan, 1.0)
        except ValueError as exc:
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
            return await asyncio.to_thread(state.replan, 1.0)
        except ValueError as exc:
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
            return await asyncio.to_thread(state.replan, 1.0)
        except ValueError as exc:
            state.log(f"当前路况下无可用路线: {exc}")
            raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/traffic/clear")
async def clear_traffic() -> dict[str, Any]:
    async with state_lock:
        state.traffic.clear_traffic()
        state.traffic_changed()
        state.log("Traffic cleared")
        try:
            return await asyncio.to_thread(state.replan, 1.0)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/replan")
async def replan(seconds: float = 1.5) -> dict[str, Any]:
    if not 0.0 < seconds <= 30.0:
        raise HTTPException(status_code=422, detail="seconds must be within (0, 30]")
    async with state_lock:
        state.log("Manual replan triggered")
        try:
            return await asyncio.to_thread(state.replan, seconds)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/v2x/auto")
async def toggle_auto(enabled: bool = True) -> dict[str, str]:
    async with state_lock:
        state.auto_v2x = enabled
        state.log(f"Auto V2X {'ON' if enabled else 'OFF'}")
        return {"auto_v2x": str(enabled)}


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
