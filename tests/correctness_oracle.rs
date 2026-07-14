//! End-to-end correctness contracts for the planners.
//!
//! The small C++ reference fake maps are intentionally used here because an
//! exact multi-objective shortest-path oracle is affordable.  This verifies
//! more than endpoint reachability: every returned edge must exist, all
//! objectives must be visited, the reported cost must equal the graph cost,
//! and no planner may beat the exact oracle.

use std::cmp::{Ordering, Reverse};
use std::collections::BinaryHeap;
use std::sync::Arc;
use std::time::Duration;

use IMOMD_RRTStar::baseline::{AnaStar, BaselinePlanner, BiAstar};
use IMOMD_RRTStar::config::{AlgorithmConfig, GaSettings, RtspSettings};
use IMOMD_RRTStar::graph::{RoadGraph, TrafficGraph, TrafficLevel};
use IMOMD_RRTStar::map::{FakeMapLoader, MapLoader};
use IMOMD_RRTStar::rrt::ImomdRrtStar;
use IMOMD_RRTStar::rtsp::EciGenSolver;
use IMOMD_RRTStar::types::{Destinations, NodeId, PlanningResult};

#[derive(Clone, Copy, Debug, PartialEq)]
struct Cost(f64);

impl Eq for Cost {}

impl PartialOrd for Cost {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Cost {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

fn config(system: u8, pseudo: u8) -> AlgorithmConfig {
    AlgorithmConfig::from_yaml_str(&format!(
        r#"
general: {{ system: {system}, pseudo: {pseudo}, log_data: 0, print_path: 0, max_iter: 50000, max_time: 30 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: 0, objective_ids: [1], target_id: 2 }}
map: {{ type: -1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 0, ga: {{ random_seed: 0, mutation_iter: 10, population: 10, generation: 1 }} }}
"#
    ))
    .expect("test configuration must parse")
}

fn dijkstra(graph: &impl RoadGraph, source: NodeId, target: NodeId) -> Option<(f64, Vec<NodeId>)> {
    let mut dist = vec![f64::INFINITY; graph.node_count()];
    let mut parent = vec![None; graph.node_count()];
    let mut open = BinaryHeap::new();
    dist[source] = 0.0;
    open.push(Reverse((Cost(0.0), source)));

    while let Some(Reverse((Cost(cost), node))) = open.pop() {
        if cost > dist[node] {
            continue;
        }
        if node == target {
            break;
        }
        for (next, weight) in graph.neighbors(node) {
            let next_cost = cost + weight;
            if next_cost < dist[next] {
                dist[next] = next_cost;
                parent[next] = Some(node);
                open.push(Reverse((Cost(next_cost), next)));
            }
        }
    }

    if !dist[target].is_finite() {
        return None;
    }
    let mut path = vec![target];
    let mut node = target;
    while node != source {
        node = parent[node]?;
        path.push(node);
    }
    path.reverse();
    Some((dist[target], path))
}

fn exact_multi_objective_cost(graph: &impl RoadGraph, destinations: &Destinations) -> f64 {
    fn visit(
        graph: &impl RoadGraph,
        current: NodeId,
        remaining: &mut Vec<NodeId>,
        target: NodeId,
    ) -> f64 {
        if remaining.is_empty() {
            return dijkstra(graph, current, target)
                .map(|(cost, _)| cost)
                .unwrap_or(f64::INFINITY);
        }

        let mut best = f64::INFINITY;
        for idx in 0..remaining.len() {
            let objective = remaining.remove(idx);
            if let Some((leg_cost, _)) = dijkstra(graph, current, objective) {
                best = best.min(leg_cost + visit(graph, objective, remaining, target));
            }
            remaining.insert(idx, objective);
        }
        best
    }

    visit(
        graph,
        destinations.source,
        &mut destinations.objectives.clone(),
        destinations.target,
    )
}

fn assert_planning_contract(
    graph: &impl RoadGraph,
    destinations: &Destinations,
    result: &PlanningResult,
    expect_optimal: bool,
) {
    assert_eq!(result.path.first().copied(), Some(destinations.source));
    assert_eq!(result.path.last().copied(), Some(destinations.target));
    for objective in &destinations.objectives {
        assert!(
            result.path.contains(objective),
            "path {:?} skipped objective {objective}",
            result.path
        );
    }

    let recomputed_cost: f64 = result
        .path
        .windows(2)
        .map(|edge| {
            graph
                .edge_weight(edge[0], edge[1])
                .unwrap_or_else(|| panic!("path contains non-edge {} -> {}", edge[0], edge[1]))
        })
        .sum();
    assert!(
        (recomputed_cost - result.cost).abs() <= 1e-3,
        "reported cost {} differs from edge sum {recomputed_cost}",
        result.cost
    );

    let oracle = exact_multi_objective_cost(graph, destinations);
    assert!(oracle.is_finite());
    assert!(
        result.cost + 1e-3 >= oracle,
        "planner cost {} is below exact oracle {oracle}",
        result.cost
    );
    if expect_optimal {
        assert!(
            (result.cost - oracle).abs() <= 1e-3,
            "small deterministic map should be optimal: got {}, oracle {oracle}",
            result.cost
        );
    }
}

#[test]
fn all_planners_match_oracle_on_fake_map_1() {
    let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
    let destinations = Destinations {
        source: 0,
        objectives: vec![1],
        target: 2,
    };

    let mut imomd =
        ImomdRrtStar::new(Arc::clone(&graph), destinations.clone(), config(0, 0)).unwrap();
    let imomd_result = imomd.run_for(Duration::from_secs(3)).unwrap();
    assert_planning_contract(graph.as_ref(), &destinations, &imomd_result, true);

    let mut bi_astar =
        BiAstar::new(Arc::clone(&graph), destinations.clone(), config(1, 0)).unwrap();
    let bi_astar_result = bi_astar.find_shortest_path().unwrap();
    assert_planning_contract(graph.as_ref(), &destinations, &bi_astar_result, true);

    let mut ana_star =
        AnaStar::new(Arc::clone(&graph), destinations.clone(), config(2, 0)).unwrap();
    let ana_star_result = ana_star.find_shortest_path().unwrap();
    assert_planning_contract(graph.as_ref(), &destinations, &ana_star_result, true);
}

#[test]
fn imomd_matches_oracle_and_visits_objective_in_bugtrap() {
    let graph = Arc::new(FakeMapLoader::new(-2).load().unwrap());
    let destinations = Destinations {
        source: 6,
        objectives: vec![2],
        target: 0,
    };
    let mut imomd =
        ImomdRrtStar::new(Arc::clone(&graph), destinations.clone(), config(0, 0)).unwrap();
    let result = imomd.run_for(Duration::from_secs(3)).unwrap();
    assert_planning_contract(graph.as_ref(), &destinations, &result, true);
}

#[test]
fn pseudo_mode_returns_a_valid_objective_route_on_bugtrap() {
    let graph = Arc::new(FakeMapLoader::new(-2).load().unwrap());
    let destinations = Destinations {
        source: 6,
        objectives: vec![2],
        target: 0,
    };
    let mut imomd =
        ImomdRrtStar::new(Arc::clone(&graph), destinations.clone(), config(0, 1)).unwrap();
    let result = imomd.run_for(Duration::from_secs(3)).unwrap();
    assert_planning_contract(graph.as_ref(), &destinations, &result, true);
}

#[test]
fn pseudo_mode_handles_multiple_objectives_on_bugtrap() {
    let graph = Arc::new(FakeMapLoader::new(-2).load().unwrap());
    let destinations = Destinations {
        source: 6,
        objectives: vec![3, 2],
        target: 0,
    };
    let mut imomd =
        ImomdRrtStar::new(Arc::clone(&graph), destinations.clone(), config(0, 1)).unwrap();
    let result = imomd.run_for(Duration::from_secs(3)).unwrap();
    assert_planning_contract(graph.as_ref(), &destinations, &result, true);
}

#[test]
fn genetic_rtsp_keeps_all_destinations_and_reports_its_own_cost() {
    let settings = RtspSettings {
        shortcut: 1,
        swapping: 1,
        genetic: 1,
        ga: GaSettings {
            random_seed: 0,
            mutation_iter: 100,
            population: 50,
            generation: 2,
        },
    };
    let matrix = vec![
        vec![0.0, 2.0, 9.0, 9.0, 20.0],
        vec![2.0, 0.0, 2.0, 8.0, 9.0],
        vec![9.0, 2.0, 0.0, 2.0, 8.0],
        vec![9.0, 8.0, 2.0, 0.0, 2.0],
        vec![20.0, 9.0, 8.0, 2.0, 0.0],
    ];
    let mut solver = EciGenSolver::new(&settings);
    let (cost, sequence) = solver.solve_rtsp(&matrix, 0, 4);

    assert_eq!(sequence.first().copied(), Some(0));
    assert_eq!(sequence.last().copied(), Some(4));
    for node in 0..matrix.len() {
        assert!(
            sequence.contains(&node),
            "GA sequence skipped destination {node}: {sequence:?}"
        );
    }
    let recomputed: f64 = sequence
        .windows(2)
        .map(|edge| matrix[edge[0]][edge[1]])
        .sum();
    assert!((cost - recomputed).abs() <= 1e-9);
}

#[test]
fn dynamic_graph_update_reuses_valid_tree_state_and_replans() {
    let base = FakeMapLoader::new(-2).load().unwrap();
    let graph = Arc::new(base.clone());
    let destinations = Destinations {
        source: 6,
        objectives: vec![2],
        target: 0,
    };
    let mut planner =
        ImomdRrtStar::new(Arc::clone(&graph), destinations.clone(), config(0, 0)).unwrap();
    let initial = planner.run_for(Duration::from_millis(100)).unwrap();
    assert_planning_contract(graph.as_ref(), &destinations, &initial, true);

    let congested_edge = [initial.path[1], initial.path[2]];
    let mut traffic = TrafficGraph::new(base);
    traffic.set_edge_level(congested_edge[0], congested_edge[1], TrafficLevel::Jam);
    let updated_graph = Arc::new(traffic.materialize().unwrap());
    let stats = planner.update_graph(Arc::clone(&updated_graph)).unwrap();

    assert!(stats.retained_tree_nodes > destinations.all_nodes().len());
    assert_eq!(
        stats.pruned_tree_nodes, 0,
        "weight-only update should retain branches"
    );

    let updated = planner.run_for(Duration::from_millis(100)).unwrap();
    assert_planning_contract(updated_graph.as_ref(), &destinations, &updated, true);
    assert!(updated.cost >= initial.cost);

    traffic.set_edge_level(congested_edge[0], congested_edge[1], TrafficLevel::Blocked);
    let blocked_graph = Arc::new(traffic.materialize().unwrap());
    let blocked_stats = planner.update_graph(Arc::clone(&blocked_graph)).unwrap();
    assert!(blocked_stats.pruned_tree_nodes > 0);

    let rerouted = planner.run_for(Duration::from_millis(100)).unwrap();
    assert_planning_contract(blocked_graph.as_ref(), &destinations, &rerouted, true);
    assert!(!rerouted.path.windows(2).any(|edge| {
        (edge[0] == congested_edge[0] && edge[1] == congested_edge[1])
            || (edge[0] == congested_edge[1] && edge[1] == congested_edge[0])
    }));
}
