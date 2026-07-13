import unittest

try:
    from IMOMD_RRTStar import FakeMap, ImomdPlanner
    from IMOMD_RRTStar import plan_fake_map
    HAS_EXTENSION = True
except ImportError:
    HAS_EXTENSION = False


@unittest.skipUnless(HAS_EXTENSION, "Rust extension not built; run maturin develop")
class TestFakeMap(unittest.TestCase):
    def test_load_map_1(self):
        g = FakeMap.load(-1)
        self.assertEqual(g.node_count, 4)

    def test_load_map_2(self):
        g = FakeMap.load(-2)
        self.assertEqual(g.node_count, 7)


@unittest.skipUnless(HAS_EXTENSION, "Rust extension not built; run maturin develop")
class TestPlannerSkeleton(unittest.TestCase):
    def test_planner_tree_count(self):
        g = FakeMap.load(-1)
        planner = ImomdPlanner(g, 0, [1], 2)
        self.assertEqual(planner.tree_count(), 3)

    def test_plan_not_implemented_yet(self):
        """Full planning returns NotImplemented until Phase 3+."""
        with self.assertRaises(NotImplementedError):
            plan_fake_map(map_type=-1, max_time_secs=0.1)


if __name__ == "__main__":
    unittest.main()
