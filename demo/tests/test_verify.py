"""Unit tests for Dijkstra path verification."""

from __future__ import annotations

import math
import unittest

from app.verify import build_graph, dijkstra, oracle_mo_cost, path_cost, verify_plan


class VerifyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.edges = [
            {"from": 0, "to": 1, "weight": 10.0},
            {"from": 1, "to": 2, "weight": 5.0},
            {"from": 0, "to": 2, "weight": 100.0},
        ]
        self.graph = build_graph(self.edges)

    def test_dijkstra_shortest(self) -> None:
        cost, path = dijkstra(self.graph, 0, 2)
        self.assertAlmostEqual(cost, 15.0)
        self.assertEqual(path, [0, 1, 2])

    def test_path_cost_recompute(self) -> None:
        total, broken = path_cost(self.graph, [0, 1, 2])
        self.assertEqual(broken, [])
        self.assertAlmostEqual(total, 15.0)

    def test_oracle_single_objective(self) -> None:
        cost, path = oracle_mo_cost(self.graph, 0, [1], 2)
        self.assertAlmostEqual(cost, 15.0)
        self.assertEqual(path, [0, 1, 2])

    def test_verify_accepts_valid_plan(self) -> None:
        report = verify_plan(
            edges=self.edges,
            path=[0, 1, 2],
            planner_cost=15.0,
            source=0,
            objectives=[1],
            target=2,
        )
        self.assertTrue(report.ok)
        self.assertAlmostEqual(report.oracle_cost or 0, 15.0)

    def test_verify_rejects_broken_edge(self) -> None:
        report = verify_plan(
            edges=self.edges,
            path=[0, 1, 99],
            planner_cost=15.0,
            source=0,
            objectives=[1],
            target=2,
        )
        self.assertFalse(report.ok)
        self.assertTrue(report.broken_edges)

    def test_oracle_skips_large_objective_sets(self) -> None:
        objs = list(range(1, 31))
        cost, path = oracle_mo_cost(self.graph, 0, objs, 2)
        self.assertIsNone(cost)
        self.assertEqual(path, [])

    def test_verify_many_objectives_does_not_hang(self) -> None:
        # Include many phantom objectives so oracle would be n!; path can't visit
        # them all — we only assert the oracle short-circuits (no hang).
        report = verify_plan(
            edges=self.edges,
            path=[0, 1, 2],
            planner_cost=15.0,
            source=0,
            objectives=list(range(10, 40)),
            target=2,
        )
        self.assertIsNone(report.oracle_cost)
        self.assertFalse(report.ok)  # path cannot visit those objectives

