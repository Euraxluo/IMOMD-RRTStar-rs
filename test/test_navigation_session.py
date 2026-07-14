"""NavigationSession Python bindings."""

from __future__ import annotations

import unittest

from IMOMD_RRTStar import FakeMap, NavigationSession, TrafficGraph


class NavigationSessionTests(unittest.TestCase):
    def test_destinations_and_continue(self) -> None:
        graph = FakeMap.load(-1)
        session = NavigationSession("imomd")
        session.set_graph(graph)
        updates = session.set_destinations(0, [1], 2, budget_secs=0.4)
        self.assertTrue(any(u.get("path") for u in updates))
        self.assertIsNotNone(session.best())
        more = session.continue_search(0.3)
        self.assertTrue(len(more) >= 1)
        self.assertEqual(session.algorithm_id, "imomd")

    def test_traffic_and_ego(self) -> None:
        traffic = TrafficGraph.load_fake(-1)
        session = NavigationSession()
        session.set_graph(traffic.materialize())
        session.set_destinations(0, [1], 2, budget_secs=0.3)
        traffic.set_edge_traffic(0, 1, "jam")
        session.set_graph(traffic.materialize())
        updates = session.on_traffic_changed(0.3)
        self.assertTrue(any(u["reason"] == "traffic_warm_start" for u in updates))
        ego = session.snap_ego(0.0, 0.0)
        self.assertEqual(ego, 0)
        moved = session.on_ego_moved(1, budget_secs=0.3)
        self.assertTrue(any(u["reason"] == "ego_reseed" for u in moved))
        self.assertEqual(session.ego_node, 1)


if __name__ == "__main__":
    unittest.main()
