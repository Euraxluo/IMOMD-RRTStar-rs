use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::baseline::{AnaStar, BaselinePlanner, BiAstar};
use crate::config::{AlgorithmConfig, OsmWayConfig};
use crate::error::{PlannerError, Result};
use crate::graph::AdjacencyGraph;
use crate::map::{CustomGraphLoader, FakeMapLoader, MapLoader, OsmMapLoader};
use crate::rrt::ImomdRrtStar;
use crate::types::{Destinations, PlannerSystem, PlanningResult};

/// Top-level planning system orchestrator (maps to C++ `main.cpp` switch).
pub struct PlanningSystem {
    graph: Arc<AdjacencyGraph>,
    config: AlgorithmConfig,
    destinations: Destinations,
}

impl PlanningSystem {
    pub fn from_config(config: AlgorithmConfig) -> Result<Self> {
        let graph = match config.map.r#type {
            t if t < 0 => {
                let loader = FakeMapLoader::new(t);
                Arc::new(loader.load()?)
            }
            0 => {
                let graph_path = std::path::Path::new(&config.map.path).join(&config.map.name);
                let loader = CustomGraphLoader::new(graph_path);
                Arc::new(loader.load()?)
            }
            1 => {
                let osm_config_path = std::path::Path::new("config/osm_way_config.yaml");
                let osm_cfg = OsmWayConfig::from_yaml_file(osm_config_path)?;
                let filter = osm_cfg.filter_properties()?;
                let osm_path = std::path::Path::new(&config.map.path).join(&config.map.name);
                let loader = OsmMapLoader::new(osm_path, filter);
                Arc::new(loader.load()?)
            }
            other => return Err(PlannerError::MapLoad(format!("invalid map type: {other}"))),
        };

        let destinations = Destinations {
            source: config.destinations.source_id,
            objectives: config.destinations.objective_ids.clone(),
            target: config.destinations.target_id,
        };

        Ok(Self {
            graph,
            config,
            destinations,
        })
    }

    pub fn from_yaml(path: &std::path::Path) -> Result<Self> {
        let config = AlgorithmConfig::from_yaml_file(path)?;
        Self::from_config(config)
    }

    pub fn print_path_enabled(&self) -> bool {
        self.config.general.print_path != 0
    }

    pub fn run(&mut self) -> Result<PlanningResult> {
        let system = PlannerSystem::from_u8(self.config.general.system)
            .ok_or_else(|| PlannerError::Config("invalid system id".into()))?;

        let max_time = Duration::from_secs(self.config.general.max_time);
        let deadline = Instant::now() + max_time;

        match system {
            PlannerSystem::Imomd => {
                let mut planner = ImomdRrtStar::new(
                    Arc::clone(&self.graph),
                    self.destinations.clone(),
                    self.config.clone(),
                )?;
                planner.run_until(deadline)
            }
            PlannerSystem::BiAstar => {
                let mut planner = BiAstar::new(
                    Arc::clone(&self.graph),
                    self.destinations.clone(),
                    self.config.clone(),
                )?;
                planner.find_shortest_path()
            }
            PlannerSystem::AnaStar => {
                let mut planner = AnaStar::new(
                    Arc::clone(&self.graph),
                    self.destinations.clone(),
                    self.config.clone(),
                )?;
                planner.find_shortest_path()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn system_from_custom_graph_config() {
        let cfg = AlgorithmConfig::from_yaml_str(
            r#"
general: { system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: 1000, max_time: 10 }
rrt_params: { goal_bias: 1.0, random_seed: 0 }
destinations: { source_id: 0, objective_ids: [1], target_id: 2 }
map: { type: 0, path: tests/fixtures/, name: custom_graph.yaml }
rtsp_settings: { shortcut: 1, swapping: 1, genetic: 0, ga: { random_seed: 0, mutation_iter: 10, population: 10, generation: 1 } }
"#,
        )
        .unwrap();
        let system = PlanningSystem::from_config(cfg);
        assert!(system.is_ok());
    }

    #[test]
    fn system_from_fake_map_config() {
        let cfg = AlgorithmConfig::from_yaml_str(
            r#"
general: { system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: 1, max_time: 1 }
rrt_params: { goal_bias: 1.0, random_seed: 0 }
destinations: { source_id: 0, objective_ids: [1], target_id: 2 }
map: { type: -1, path: "", name: "" }
rtsp_settings: { shortcut: 1, swapping: 1, genetic: 1, ga: { random_seed: 0, mutation_iter: 10, population: 10, generation: 1 } }
"#,
        )
        .unwrap();
        let system = PlanningSystem::from_config(cfg);
        assert!(system.is_ok());
    }

    #[test]
    fn print_path_flag_is_exposed_to_cli() {
        let cfg = AlgorithmConfig::from_yaml_str(
            r#"
general: { system: 0, pseudo: 0, log_data: 0, print_path: 1, max_iter: 1, max_time: 1 }
rrt_params: { goal_bias: 1.0, random_seed: 0 }
destinations: { source_id: 0, objective_ids: [1], target_id: 2 }
map: { type: -1, path: "", name: "" }
rtsp_settings: { shortcut: 1, swapping: 1, genetic: 0, ga: { random_seed: 0, mutation_iter: 10, population: 10, generation: 1 } }
"#,
        )
        .unwrap();
        let system = PlanningSystem::from_config(cfg).unwrap();
        assert!(system.print_path_enabled());
    }
}
