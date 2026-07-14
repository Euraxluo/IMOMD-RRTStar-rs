import unittest
import ast
from pathlib import Path

try:
    from IMOMD_RRTStar import (
        AlgorithmConfig,
        CustomGraph,
        FakeMap,
        ImomdPlanner,
        OsmMap,
        TrafficGraph,
    )
    from IMOMD_RRTStar import plan_fake_map
    HAS_EXTENSION = True
except ImportError:
    HAS_EXTENSION = False

FIXTURES = Path(__file__).resolve().parent.parent / "tests" / "fixtures"


@unittest.skipUnless(HAS_EXTENSION, "Rust extension not built; run maturin develop")
class TestFakeMap(unittest.TestCase):
    def test_load_map_1(self):
        g = FakeMap.load(-1)
        self.assertEqual(g.node_count, 4)

    def test_load_map_2(self):
        g = FakeMap.load(-2)
        self.assertEqual(g.node_count, 7)

    def test_load_custom_graph(self):
        g = CustomGraph.load(str(FIXTURES / "custom_graph.yaml"))
        self.assertEqual(g.node_count, 4)

    def test_load_osm_map(self):
        g = OsmMap.load(str(FIXTURES / "tiny.osm"))
        self.assertEqual(g.node_count, 3)

    def test_traffic_graph_exports_and_materializes(self):
        traffic = TrafficGraph.load_fake(-1)
        view = traffic.export_view()
        self.assertEqual(len(view["nodes"]), 4)
        self.assertTrue(view["edges"])

        edge = view["edges"][0]
        traffic.set_edge_traffic(edge["from"], edge["to"], "jam")
        jammed = next(
            item
            for item in traffic.export_view()["edges"]
            if {item["from"], item["to"]} == {edge["from"], edge["to"]}
        )
        self.assertEqual(jammed["level"], "jam")
        self.assertGreater(jammed["weight"], jammed["base_weight"])
        self.assertEqual(traffic.materialize().node_count, 4)

        traffic.clear_traffic()
        self.assertTrue(all(item["level"] == "free" for item in traffic.export_view()["edges"]))


@unittest.skipUnless(HAS_EXTENSION, "Rust extension not built; run maturin develop")
class TestPlannerSkeleton(unittest.TestCase):
    def test_planner_tree_count(self):
        g = FakeMap.load(-1)
        planner = ImomdPlanner(g, 0, [1], 2)
        self.assertEqual(planner.tree_count(), 3)
        self.assertIsNone(planner.best_result())
        step = planner.step()
        self.assertIn(step["status"], {"expanded", "connected", "path_improved", "finished"})
        self.assertGreaterEqual(step["iteration"], 1)

    def test_plan_fake_map_1(self):
        result = plan_fake_map(map_type=-1, max_time_secs=1.0)
        self.assertEqual(result.path[0], 0)
        self.assertEqual(result.path[-1], 2)
        self.assertGreater(result.cost, 0.0)

    def test_config_constructor_round_trip_and_anytime_completion(self):
        cfg = AlgorithmConfig.from_yaml(str(Path(__file__).resolve().parent.parent / "config" / "algorithm_config.yaml"))
        round_trip = AlgorithmConfig.from_yaml_string(cfg.to_yaml_string())
        self.assertEqual(round_trip.objectives, [1])

        planner = ImomdPlanner(FakeMap.load(-1), round_trip)
        first = planner.run_for(0.01)
        second = planner.run_for(0.01)
        self.assertEqual(first.path, second.path)
        self.assertAlmostEqual(first.cost, second.cost)
        if planner.is_finished:
            self.assertEqual(planner.step()["status"], "finished")

    def test_dynamic_graph_update_reuses_tree_state(self):
        traffic = TrafficGraph.load_fake(-2)
        graph = traffic.materialize()
        planner = ImomdPlanner(graph, 6, [2], 0)
        initial = planner.run_for(0.1)

        traffic.set_edge_traffic(initial.path[1], initial.path[2], "jam")
        stats = planner.update_graph(traffic.materialize())
        self.assertGreater(stats.retained_tree_nodes, 3)
        self.assertEqual(stats.pruned_tree_nodes, 0)

        updated = planner.run_until(0.1)
        self.assertEqual(updated.path[0], 6)
        self.assertEqual(updated.path[-1], 0)

    def test_python_argument_validation(self):
        planner = ImomdPlanner(FakeMap.load(-1), 0, [1], 2)
        with self.assertRaises(ValueError):
            planner.run_for(0.0)
        traffic = TrafficGraph.load_fake(-1)
        with self.assertRaises(ValueError):
            traffic.set_edge_traffic(0, 1, "teleporting")
        with self.assertRaises(ValueError):
            traffic.set_edge_traffic(0, 2, "jam")
        with self.assertRaises(ValueError):
            traffic.set_zone_traffic([999], "slow")


class TestTypingPackage(unittest.TestCase):
    def test_pep561_stub_and_marker_are_valid(self):
        package_dir = Path(__file__).resolve().parents[1] / "python" / "IMOMD_RRTStar"
        stub = package_dir / "__init__.pyi"
        marker = package_dir / "py.typed"
        self.assertTrue(marker.is_file())
        ast.parse(stub.read_text(encoding="utf-8"), filename=str(stub))
        declarations = stub.read_text(encoding="utf-8")
        for name in ("CustomGraph", "OsmMap", "TrafficGraph", "plan_fake_map"):
            self.assertIn(name, declarations)


if __name__ == "__main__":
    unittest.main()
