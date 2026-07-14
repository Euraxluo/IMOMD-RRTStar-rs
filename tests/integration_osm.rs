use std::collections::VecDeque;
use std::path::Path;

use IMOMD_RRTStar::config::OsmWayConfig;
use IMOMD_RRTStar::graph::RoadGraph;
use IMOMD_RRTStar::map::OsmMapLoader;

fn frb_osm_path() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tmp/imomd-cpp/osm_data/FRB.osm")
}

fn load_frb_graph() -> Option<IMOMD_RRTStar::graph::AdjacencyGraph> {
    let osm_path = frb_osm_path();
    if !osm_path.exists() {
        return None;
    }
    let filter = OsmWayConfig::from_yaml_file(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("config/osm_way_config.yaml"),
    )
    .ok()?
    .filter_properties()
    .ok()?;
    OsmMapLoader::parse_file(&osm_path, &filter).ok()
}

/// BFS distances from `start` across the road graph.
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

/// Pick spread-out reachable destinations for smoke tests on OSM maps.
pub fn pick_spread_destinations(graph: &impl RoadGraph) -> Option<(usize, Vec<usize>, usize)> {
    let n = graph.node_count();
    if n < 4 {
        return None;
    }

    let start = (0..n).find(|&i| !graph.neighbors(i).is_empty())?;
    let dist = bfs_distances(graph, start);
    let reachable: Vec<usize> = (0..n).filter(|&i| dist[i].is_some()).collect();
    if reachable.len() < 4 {
        return None;
    }

    let pick = |frac: f64| {
        let idx = ((reachable.len() as f64 - 1.0) * frac).round() as usize;
        reachable[idx.min(reachable.len() - 1)]
    };

    let source = start;
    let target = *reachable.last().unwrap();
    let objectives = vec![pick(0.33), pick(0.66)];
    Some((source, objectives, target))
}

#[test]
fn integration_frb_osm_loads() {
    let Some(graph) = load_frb_graph() else {
        return;
    };
    assert!(graph.node_count() > 100);
}

#[test]
fn integration_frb_osm_planner_finds_path() {
    use std::sync::Arc;
    use std::time::Duration;

    use IMOMD_RRTStar::config::AlgorithmConfig;
    use IMOMD_RRTStar::rrt::ImomdRrtStar;
    use IMOMD_RRTStar::types::Destinations;

    let Some(graph) = load_frb_graph() else {
        return;
    };

    let (source, objectives, target) = pick_spread_destinations(&graph).expect("connected nodes");
    let config = AlgorithmConfig::from_yaml_str(&format!(
        r#"
general: {{ system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: 50000, max_time: 30 }}
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
    let result = planner.run_for(Duration::from_secs(15)).unwrap();

    assert_eq!(result.path.first().copied(), Some(source));
    assert_eq!(result.path.last().copied(), Some(target));
    assert!(result.cost.is_finite() && result.cost > 0.0);
    assert!(!result.path.is_empty());
}
