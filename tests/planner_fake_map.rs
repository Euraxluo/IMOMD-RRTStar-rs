use std::sync::Arc;
use std::time::Duration;

use IMOMD_RRTStar::baseline::{AnaStar, BaselinePlanner, BiAstar};
use IMOMD_RRTStar::config::AlgorithmConfig;
use IMOMD_RRTStar::map::{FakeMapLoader, MapLoader};
use IMOMD_RRTStar::rrt::ImomdRrtStar;
use IMOMD_RRTStar::types::Destinations;

fn planner_config(system: u8, max_iter: usize) -> AlgorithmConfig {
    AlgorithmConfig::from_yaml_str(&format!(
        r#"
general: {{ system: {system}, pseudo: 0, log_data: 0, print_path: 0, max_iter: {max_iter}, max_time: 30 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: 6, objective_ids: [2], target_id: 0 }}
map: {{ type: -2, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 0, ga: {{ random_seed: 0, mutation_iter: 10, population: 10, generation: 1 }} }}
"#
    ))
    .unwrap()
}

fn bugtrap_destinations() -> Destinations {
    Destinations {
        source: 6,
        objectives: vec![2],
        target: 0,
    }
}

#[test]
fn integration_fake_map_2_finds_bugtrap_path() {
    let graph = Arc::new(FakeMapLoader::new(-2).load().unwrap());
    let mut planner =
        ImomdRrtStar::new(graph, bugtrap_destinations(), planner_config(0, 50_000)).unwrap();
    let result = planner.run_for(Duration::from_secs(10)).unwrap();
    assert_valid_bugtrap_result(&result.path, result.cost);
}

#[test]
fn integration_bi_astar_finds_bugtrap_path() {
    let graph = Arc::new(FakeMapLoader::new(-2).load().unwrap());
    let mut planner =
        BiAstar::new(graph, bugtrap_destinations(), planner_config(1, 50_000)).unwrap();
    let result = planner.find_shortest_path().unwrap();
    assert_valid_bugtrap_result(&result.path, result.cost);
}

#[test]
fn integration_ana_star_finds_bugtrap_path() {
    let graph = Arc::new(FakeMapLoader::new(-2).load().unwrap());
    let mut planner =
        AnaStar::new(graph, bugtrap_destinations(), planner_config(2, 50_000)).unwrap();
    let result = planner.find_shortest_path().unwrap();
    assert_valid_bugtrap_result(&result.path, result.cost);
}

fn assert_valid_bugtrap_result(path: &[usize], cost: f64) {
    assert_eq!(*path.first().unwrap(), 6);
    assert_eq!(*path.last().unwrap(), 0);
    assert!(cost > 0.0 && cost.is_finite());
    assert!(path.windows(2).all(|w| w[0] != w[1]));
}
