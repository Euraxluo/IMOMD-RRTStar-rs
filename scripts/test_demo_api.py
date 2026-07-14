#!/usr/bin/env python3
"""Smoke-test the V2X demo FastAPI backend (server must be running)."""

from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8000"


def get(path: str) -> dict:
    with urllib.request.urlopen(f"{BASE}{path}", timeout=30) as resp:
        return json.loads(resp.read())


def post(path: str, body: dict | None = None) -> dict:
    data = json.dumps(body).encode() if body is not None else b"null"
    req = urllib.request.Request(
        f"{BASE}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        return json.loads(resp.read())


def post_error(path: str, body: dict | None = None) -> tuple[int, dict]:
    try:
        post(path, body)
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())
    raise AssertionError(f"expected {path} to fail")


def main() -> int:
    try:
        state = get("/api/state")
    except urllib.error.URLError as exc:
        print(f"Demo server not reachable at {BASE}: {exc}", file=sys.stderr)
        return 1

    assert "map_name" in state and state["node_count"] > 0
    assert state.get("map_key") == "city_large", state.get("map_key")
    assert state["node_count"] >= 400, state["node_count"]
    v = state.get("verification") or {}
    print(f"map={state['map_name']} nodes={state['node_count']} cost={state.get('cost')}")
    print(f"verify ok={v.get('ok')} oracle={v.get('oracle_cost')} msg={v.get('message')}")
    assert v.get("ok"), f"verification failed: {v.get('message')}"

    status, invalid = post_error(
        "/api/traffic/edge", {"from": 0, "to": 0, "level": "jam"}
    )
    assert status == 400 and "not connected" in invalid.get("detail", ""), invalid

    dest = state["destinations"]
    zone = post("/api/traffic/zone", {"nodes": [dest["source"]], "level": "slow"})
    assert zone.get("path"), "expected path after zone traffic"
    assert zone.get("replan_mode") == "warm_start", zone
    update = zone.get("tree_update") or {}
    assert update.get("retained_tree_nodes", 0) > 0, update
    print(f"zone replan cost={zone['cost']:.1f}")

    verify = get("/api/verify")
    assert verify.get("path_valid"), verify.get("message")
    print(f"after zone verify ok={verify.get('ok')}")

    state = get("/api/state")
    path_edges = list(zip(state["path"], state["path"][1:]))[:8]
    selected = [{"from": a, "to": b} for a, b in path_edges]
    lanes = post("/api/traffic/edges", {"edges": selected, "level": "jam"})
    assert lanes.get("path"), "expected path after selected lane traffic"
    assert lanes.get("replan_mode") == "warm_start", lanes
    print(f"selected-lane replan cost={lanes['cost']:.1f}")

    cleared = post("/api/traffic/clear")
    print(f"clear replan cost={cleared['cost']:.1f}")

    post("/api/v2x/auto?enabled=false")
    before_tick = get("/api/state")["v2x_tick"]
    post("/api/v2x/auto?enabled=true")
    deadline = time.monotonic() + 5.0
    toggled = get("/api/state")
    while toggled["v2x_tick"] == before_tick and time.monotonic() < deadline:
        time.sleep(0.1)
        toggled = get("/api/state")
    post("/api/v2x/auto?enabled=false")
    assert toggled["auto_v2x"] is True
    assert toggled["v2x_tick"] == before_tick + 1, toggled
    print(f"single V2X scheduler tick={toggled['v2x_tick']}")

    print("demo API smoke OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
