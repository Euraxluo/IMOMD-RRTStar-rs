"""
V2X Demo — browser user-story tests (Playwright).

Requires demo server: ./demo/run.sh  (http://127.0.0.1:8000)
Run: uv run pytest demo/tests/test_user_stories.py -v
"""

from __future__ import annotations

import os
import re
import time

import pytest
from playwright.sync_api import Page, expect

BASE = os.environ.get("DEMO_BASE", "http://127.0.0.1:8000")


@pytest.fixture(scope="session")
def base_url() -> str:
    import urllib.error
    import urllib.request

    try:
        urllib.request.urlopen(f"{BASE}/api/state", timeout=5)
    except urllib.error.URLError as exc:
        pytest.skip(f"Demo server not running at {BASE}: {exc}")
    return BASE


@pytest.fixture
def page(page: Page, base_url: str) -> Page:
    page.goto(base_url)
    page.wait_for_selector("#stats >> text=地图")
    return page


def _cost_from_stats(page: Page) -> float | None:
    text = page.locator("#stats").inner_text()
    m = re.search(r"路径代价\s+([\d.]+)", text)
    return float(m.group(1)) if m else None


def _canvas_points(page: Page, node_ids: list[int]) -> dict[int, dict[str, float]]:
    """Return CSS-pixel click positions for graph nodes."""
    return page.evaluate(
        """async (nodeIds) => {
          const s = await fetch('/api/state').then(r => r.json());
          const nodes = s.view.nodes;
          const bounds = {
            minLat: Math.min(...nodes.map(n => n.lat)),
            maxLat: Math.max(...nodes.map(n => n.lat)),
            minLon: Math.min(...nodes.map(n => n.lon)),
            maxLon: Math.max(...nodes.map(n => n.lon)),
          };
          const canvas = document.getElementById('map');
          const rect = canvas.getBoundingClientRect();
          const pos = (id) => {
            const n = nodes.find(x => x.id === id);
            const x = ((n.lon - bounds.minLon) / (bounds.maxLon - bounds.minLon || 1)) * (canvas.width - 40) + 20;
            const y = ((bounds.maxLat - n.lat) / (bounds.maxLat - bounds.minLat || 1)) * (canvas.height - 40) + 20;
            return { x: x * rect.width / canvas.width, y: y * rect.height / canvas.height };
          };
          return Object.fromEntries(nodeIds.map(id => [id, pos(id)]));
        }""",
        node_ids,
    )


class TestUserStory01InitialLoad:
    """US-01: 作为调度员，打开页面后应看到地图、初始路径代价和自动同步的目的地。"""

    def test_page_title_and_map(self, page: Page) -> None:
        expect(page).to_have_title(re.compile("IMOMD-RRT"))
        expect(page.locator("canvas#map")).to_be_visible()
        expect(page.locator("#stats")).to_contain_text("节点")

    def test_initial_plan_has_cost(self, page: Page) -> None:
        cost = _cost_from_stats(page)
        assert cost is not None and cost > 0

    def test_destinations_synced_to_inputs(self, page: Page) -> None:
        destination = page.evaluate(
            "fetch('/api/state').then(r => r.json()).then(s => s.destinations)"
        )
        src = page.locator("#src").input_value()
        obj = page.locator("#obj").input_value()
        tgt = page.locator("#tgt").input_value()
        assert int(src) == destination["source"]
        assert int(obj) == destination["objectives"][0]
        assert int(tgt) == destination["target"]

    def test_two_route_legs_have_distinct_colors(self, page: Page) -> None:
        colors = page.evaluate("window.__IMOMD_DEMO__.pathLegColors")
        assert colors[:2] == ["#00d4ff", "#f472b6"]


class TestUserStory02ManualReplan:
    """US-02: 作为调度员，点击「立即重规划」后路径代价应仍然有效。"""

    def test_replan_button(self, page: Page) -> None:
        before = _cost_from_stats(page)
        page.locator("#btn-replan").click()
        page.wait_for_timeout(2000)
        after = _cost_from_stats(page)
        assert after is not None and after > 0
        assert before is not None


class TestUserStory03TrafficClear:
    """US-03: 作为路况管理员，清除路况后应恢复基础路网规划。"""

    def test_clear_traffic(self, page: Page) -> None:
        page.locator("#btn-clear").click()
        page.wait_for_timeout(1500)
        cost = _cost_from_stats(page)
        assert cost is not None and cost > 0
        events = page.locator("#events li").all_inner_texts()
        assert any("Traffic cleared" in e or "清除" in e or "cleared" in e.lower() for e in events)


class TestUserStory04AutoV2x:
    """US-04: 作为 V2X 运营方，开启模拟后应收到广播事件并自动重规划。"""

    def test_enable_auto_v2x(self, page: Page) -> None:
        page.locator("#btn-auto-on").click()
        page.wait_for_timeout(500)
        expect(page.locator("#stats")).to_contain_text("V2X 模拟")
        expect(page.locator("#stats")).to_contain_text("运行中")

        # WebSocket pushes every ~3s
        page.wait_for_timeout(4500)
        events = page.locator("#events li").all_inner_texts()
        assert any("V2X" in e for e in events), f"expected V2X events, got {events[:3]}"

        page.locator("#btn-auto-off").click()
        page.wait_for_timeout(300)
        expect(page.locator("#stats")).to_contain_text("已停止")


class TestUserStory05UpdateDestinations:
    """US-05: 作为路径规划师，修改起终点后应得到新路径。"""

    def test_update_destinations(self, page: Page) -> None:
        src = page.locator("#src").input_value()
        tgt = page.locator("#tgt").input_value()
        obj = page.locator("#obj").input_value()

        page.locator("details.advanced").click()
        page.locator("#btn-dest").click()
        page.wait_for_timeout(2000)
        cost = _cost_from_stats(page)
        assert cost is not None and cost > 0

        destination = page.evaluate(
            "fetch('/api/state').then(r => r.json()).then(s => s.destinations)"
        )
        assert int(src) == destination["source"]
        assert int(obj) == destination["objectives"][0]
        assert int(tgt) == destination["target"]

    def test_three_click_route_wizard_survives_heartbeat(self, page: Page) -> None:
        destination = page.evaluate(
            "fetch('/api/state').then(r => r.json()).then(s => s.destinations)"
        )
        source = destination["source"]
        objective = destination["objectives"][0]
        target = destination["target"]
        points = _canvas_points(page, [source, objective, target])
        canvas = page.locator("canvas#map")

        canvas.click(position=points[str(source)])
        expect(page.locator("#route-step")).to_contain_text("第 2 步")
        expect(page.locator("#badge-src")).to_contain_text(f"#{source}")

        # A WebSocket heartbeat must not erase an unfinished selection.
        page.wait_for_timeout(3300)
        expect(page.locator("#route-step")).to_contain_text("第 2 步")
        expect(page.locator("#badge-src")).to_contain_text(f"#{source}")

        canvas.click(position=points[str(objective)])
        expect(page.locator("#badge-obj")).to_contain_text(f"#{objective}")
        expect(page.locator("#btn-finish-objectives")).to_be_visible()
        page.locator("#btn-finish-objectives").click()
        expect(page.locator("#route-step")).to_contain_text("第 3 步")

        canvas.click(position=points[str(target)])
        expect(page.locator("#route-step")).to_contain_text("正在重规划")
        expect(page.locator("#stats")).to_contain_text("校验", timeout=5000)
        expect(page.locator("#stats")).to_contain_text("✓")


class TestUserStory06PolygonLaneTraffic:
    """US-06: 作为路况管理员，框选 lane 设置拥堵后路径应重规划。"""

    def test_polygon_lane_traffic(self, page: Page) -> None:
        before = _cost_from_stats(page)
        assert before is not None

        polygon = page.evaluate(
            """async () => {
              const res = await fetch('/api/state');
              const s = await res.json();
              const path = s.path || [];
              if (path.length < 3) return null;
              const nodes = s.view.nodes;
              const bounds = {
                minLat: Math.min(...nodes.map(n => n.lat)),
                maxLat: Math.max(...nodes.map(n => n.lat)),
                minLon: Math.min(...nodes.map(n => n.lon)),
                maxLon: Math.max(...nodes.map(n => n.lon)),
              };
              const canvas = document.getElementById('map');
              const rect = canvas.getBoundingClientRect();
              const pos = (id) => {
                const n = nodes.find(x => x.id === id);
                const x = ((n.lon - bounds.minLon) / (bounds.maxLon - bounds.minLon || 1)) * (canvas.width - 40) + 20;
                const y = ((bounds.maxLat - n.lat) / (bounds.maxLat - bounds.minLat || 1)) * (canvas.height - 40) + 20;
                return { x: x * rect.width / canvas.width, y: y * rect.height / canvas.height };
              };
              const pts = path.slice(0, Math.min(path.length, 6)).map(pos);
              const xs = pts.map(p => p.x);
              const ys = pts.map(p => p.y);
              const pad = 24;
              return [
                { x: Math.max(2, Math.min(...xs) - pad), y: Math.max(2, Math.min(...ys) - pad) },
                { x: Math.min(rect.width - 2, Math.max(...xs) + pad), y: Math.max(2, Math.min(...ys) - pad) },
                { x: Math.min(rect.width - 2, Math.max(...xs) + pad), y: Math.min(rect.height - 2, Math.max(...ys) + pad) },
                { x: Math.max(2, Math.min(...xs) - pad), y: Math.min(rect.height - 2, Math.max(...ys) + pad) },
              ];
            }"""
        )
        if polygon is None:
            pytest.skip("no usable path on map")

        page.locator("#mode-traffic").click()
        page.select_option("#level", "jam")
        canvas = page.locator("canvas#map")
        for point in polygon:
            canvas.click(position=point)
            page.wait_for_timeout(100)
        expect(page.locator("#selected-lane-count")).to_contain_text("lane")
        page.locator("#btn-traffic-apply").click()
        page.wait_for_timeout(2500)

        after = _cost_from_stats(page)
        events = page.locator("#events li").all_inner_texts()
        assert any("selected lanes" in e.lower() or "lane" in e.lower() for e in events)
        assert after is not None
        # Cost may increase or path reroute; at minimum replan succeeded
        assert after > 0
