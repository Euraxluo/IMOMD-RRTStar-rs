use std::path::Path;

use IMOMD_RRTStar::prelude::PlanningSystem;

#[test]
fn integration_custom_graph_planner_finds_path() {
    let config_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("config/algorithm_config_custom.yaml");
    let mut system = PlanningSystem::from_yaml(&config_path).unwrap();
    let result = system.run().unwrap();

    assert_eq!(result.path.first().copied(), Some(0));
    assert_eq!(result.path.last().copied(), Some(2));
    assert!(result.cost.is_finite() && result.cost > 0.0);
}
