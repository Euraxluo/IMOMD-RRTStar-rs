use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use IMOMD_RRTStar::baseline::{AnaStar, BaselinePlanner, BiAstar};
use IMOMD_RRTStar::config::AlgorithmConfig;
use IMOMD_RRTStar::experiment::{costs_are_non_increasing, final_finite_cost, parse_cpp_csv};
use IMOMD_RRTStar::map::{FakeMapLoader, MapLoader};
use IMOMD_RRTStar::rrt::ImomdRrtStar;
use IMOMD_RRTStar::types::{Destinations, StepStatus};

fn bugtrap_config(max_iter: usize, log_data: u8) -> AlgorithmConfig {
    AlgorithmConfig::from_yaml_str(&format!(
        r#"
general: {{ system: 0, pseudo: 0, log_data: {log_data}, print_path: 0, max_iter: {max_iter}, max_time: 30 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: 6, objective_ids: [2], target_id: 0 }}
map: {{ type: -2, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 0, ga: {{ random_seed: 0, mutation_iter: 10, population: 10, generation: 1 }} }}
"#
    ))
    .unwrap()
}

fn baseline_config(system: u8) -> AlgorithmConfig {
    AlgorithmConfig::from_yaml_str(&format!(
        r#"
general: {{ system: {system}, pseudo: 0, log_data: 1, print_path: 0, max_iter: 50000, max_time: 30 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: 0, objective_ids: [1], target_id: 2 }}
map: {{ type: -1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 0, ga: {{ random_seed: 0, mutation_iter: 10, population: 10, generation: 1 }} }}
"#
    ))
    .unwrap()
}

#[test]
fn cpp_bugtrap_reference_csv_is_monotonic() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tmp/imomd-cpp/experiments/bugtrap/sanfrancisco/imomd.csv");
    if !path.exists() {
        return;
    }

    let records = parse_cpp_csv(&path).unwrap();
    assert!(costs_are_non_increasing(&records));
    assert!(final_finite_cost(&records).unwrap().is_finite());
}

#[test]
fn rust_bugtrap_imomd_costs_are_monotonic() {
    let graph = Arc::new(FakeMapLoader::new(-2).load().unwrap());
    let dest = Destinations {
        source: 6,
        objectives: vec![2],
        target: 0,
    };

    let mut planner = ImomdRrtStar::new(graph, dest, bugtrap_config(50_000, 1)).unwrap();
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while std::time::Instant::now() < deadline {
        if matches!(planner.step().unwrap().status, StepStatus::Finished) {
            break;
        }
    }

    let records = planner.experiment_records();
    assert!(
        !records.is_empty(),
        "expected at least one path improvement log"
    );
    assert!(costs_are_non_increasing(records));
    assert!(final_finite_cost(records).unwrap().is_finite());
}

#[test]
fn baseline_log_data_records_path_improvements() {
    let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
    let destinations = Destinations {
        source: 0,
        objectives: vec![1],
        target: 2,
    };

    let mut bi_astar =
        BiAstar::new(Arc::clone(&graph), destinations.clone(), baseline_config(1)).unwrap();
    bi_astar.find_shortest_path().unwrap();
    assert!(!bi_astar.experiment_records().is_empty());
    assert!(costs_are_non_increasing(bi_astar.experiment_records()));

    let mut ana_star = AnaStar::new(graph, destinations, baseline_config(2)).unwrap();
    ana_star.find_shortest_path().unwrap();
    assert!(!ana_star.experiment_records().is_empty());
    assert!(costs_are_non_increasing(ana_star.experiment_records()));
}

#[test]
fn cpp_seattle_reference_csv_is_monotonic() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tmp/imomd-cpp/experiments/large/seattle/imomd.csv");
    if !path.exists() {
        return;
    }

    let records = parse_cpp_csv(&path).unwrap();
    assert!(records.len() > 10);
    assert!(costs_are_non_increasing(&records));
    assert!(final_finite_cost(&records).unwrap() < 400_000.0);
}
