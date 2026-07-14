use std::collections::VecDeque;
use std::path::Path;

use IMOMD_RRTStar::config::{AlgorithmConfig, OsmWayConfig};
use IMOMD_RRTStar::graph::RoadGraph;
use IMOMD_RRTStar::map::OsmMapLoader;

fn osm_filter() -> IMOMD_RRTStar::config::OsmFilterProperties {
    OsmWayConfig::from_yaml_file(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("config/osm_way_config.yaml"),
    )
    .unwrap()
    .filter_properties()
    .unwrap()
}

fn load_osm(name: &str) -> Option<IMOMD_RRTStar::graph::AdjacencyGraph> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tmp/imomd-cpp/osm_data")
        .join(name);
    if !path.exists() {
        return None;
    }
    OsmMapLoader::parse_file(&path, &osm_filter()).ok()
}

fn bfs_distances(graph: &impl RoadGraph, start: usize) -> Vec<Option<usize>> {
    let n = graph.node_count();
    let mut dist = vec![None; n];
    let mut queue = VecDeque::new();
    dist[start] = Some(0);
    queue.push_back(start);
    while let Some(node) = queue.pop_front() {
        let d = dist[node].unwrap();
        for (neighbor, _) in graph.neighbors(node) {
            if dist[neighbor].is_none() {
                dist[neighbor] = Some(d + 1);
                queue.push_back(neighbor);
            }
        }
    }
    dist
}

/// Pick `objective_count` spread-out objectives between source and target.
fn pick_many_spread_destinations(
    graph: &impl RoadGraph,
    objective_count: usize,
) -> Option<(usize, Vec<usize>, usize)> {
    let n = graph.node_count();
    if n < objective_count + 2 {
        return None;
    }

    let start = (0..n).find(|&i| !graph.neighbors(i).is_empty())?;
    let dist = bfs_distances(graph, start);
    let reachable: Vec<usize> = (0..n).filter(|&i| dist[i].is_some()).collect();
    if reachable.len() < objective_count + 2 {
        return None;
    }

    let source = start;
    let target = *reachable.last().unwrap();
    let objectives = (1..=objective_count)
        .map(|i| {
            let frac = i as f64 / (objective_count as f64 + 1.0);
            let idx = ((reachable.len() as f64 - 1.0) * frac).round() as usize;
            reachable[idx.min(reachable.len() - 1)]
        })
        .collect();

    Some((source, objectives, target))
}

#[test]
fn integration_quincy_multi_objective_planner_finds_path() {
    use std::sync::Arc;
    use std::time::Duration;

    use IMOMD_RRTStar::rrt::ImomdRrtStar;
    use IMOMD_RRTStar::types::Destinations;

    let Some(graph) = load_osm("quincy.osm") else {
        eprintln!("skip: quincy.osm not present (run scripts/download_osm_maps.py)");
        return;
    };

    assert!(graph.node_count() > 1_000, "expected a large road network");

    let (source, objectives, target) =
        pick_many_spread_destinations(&graph, 8).expect("enough reachable nodes");
    assert_eq!(objectives.len(), 8);

    let config = AlgorithmConfig::from_yaml_str(&format!(
        r#"
general: {{ system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: 100000, max_time: 20 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: {source}, objective_ids: {objectives:?}, target_id: {target} }}
map: {{ type: 1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 0, ga: {{ random_seed: 0, mutation_iter: 10, population: 10, generation: 1 }} }}
"#
    ))
    .unwrap();

    let dest = Destinations {
        source,
        objectives,
        target,
    };

    let mut planner = ImomdRrtStar::new(Arc::new(graph), dest, config).unwrap();
    let result = planner.run_for(Duration::from_secs(20)).unwrap();

    assert_eq!(result.path.first().copied(), Some(source));
    assert_eq!(result.path.last().copied(), Some(target));
    assert!(result.cost.is_finite() && result.cost > 0.0);
    assert!(result.explored_nodes > 100);
}

/// Seattle-scale reproduction (23 objectives + GA). Run manually:
/// `cargo test --test integration_large_scale integration_quincy_seattle_scale -- --ignored --nocapture`
#[test]
#[ignore = "slow: ~2min, mirrors C++ Seattle experiment settings"]
fn integration_quincy_seattle_scale_planner_finds_path() {
    use std::sync::Arc;
    use std::time::Duration;

    use IMOMD_RRTStar::rrt::ImomdRrtStar;
    use IMOMD_RRTStar::types::Destinations;

    let Some(graph) = load_osm("quincy.osm") else {
        eprintln!("skip: quincy.osm not present (run scripts/download_osm_maps.py)");
        return;
    };

    assert!(graph.node_count() > 1_000, "expected a large road network");

    let (source, objectives, target) =
        pick_many_spread_destinations(&graph, 23).expect("enough reachable nodes");
    assert_eq!(objectives.len(), 23);

    let config = AlgorithmConfig::from_yaml_str(&format!(
        r#"
general: {{ system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: 200000, max_time: 60 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: {source}, objective_ids: {objectives:?}, target_id: {target} }}
map: {{ type: 1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 1, ga: {{ random_seed: 0, mutation_iter: 100, population: 100, generation: 2 }} }}
"#
    ))
    .unwrap();

    let dest = Destinations {
        source,
        objectives,
        target,
    };

    let mut planner = ImomdRrtStar::new(Arc::new(graph), dest, config).unwrap();
    let result = planner.run_for(Duration::from_secs(30)).unwrap();

    assert_eq!(result.path.first().copied(), Some(source));
    assert_eq!(result.path.last().copied(), Some(target));
    assert!(result.cost.is_finite() && result.cost > 0.0);
    assert!(result.explored_nodes > 100);
}

#[test]
fn integration_seattle_osm_planner_when_available() {
    use std::path::Path;

    use IMOMD_RRTStar::prelude::PlanningSystem;

    let seattle = Path::new(env!("CARGO_MANIFEST_DIR")).join("tmp/imomd-cpp/osm_data/Seattle.osm");
    if !seattle.exists() {
        eprintln!("skip: Seattle.osm not present (run scripts/download_osm_maps.py --drive)");
        return;
    }

    let config_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("config/algorithm_config_seattle.yaml");
    let mut system = PlanningSystem::from_yaml(&config_path).unwrap();
    let result = system.run().unwrap();

    assert!(result.cost.is_finite() && result.cost > 0.0);
    assert!(!result.path.is_empty());
}
