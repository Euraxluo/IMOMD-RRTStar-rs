use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::config::AlgorithmConfig;
use crate::error::{PlannerError, Result};
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::types::{Destinations, PlanningResult, StepResult, StepStatus};
use crate::rrt::tree::RrtTree;

/// Anytime IMOMD-RRT* planner (maps to C++ `ImomdRRT`).
pub struct ImomdRrtStar {
    graph: Arc<AdjacencyGraph>,
    destinations: Destinations,
    config: AlgorithmConfig,
    iteration: usize,
    best: Option<PlanningResult>,
    finished: bool,
    // TODO: tree_layers_, distance_matrix_, eci_gen_solver_, etc.
}

impl ImomdRrtStar {
    pub fn new(
        graph: Arc<AdjacencyGraph>,
        destinations: Destinations,
        config: AlgorithmConfig,
    ) -> Result<Self> {
        for &node in destinations.all_nodes().iter() {
            if graph.location(node).is_none() {
                return Err(PlannerError::NodeNotFound(node));
            }
        }
        Ok(Self {
            graph,
            destinations,
            config,
            iteration: 0,
            best: None,
            finished: false,
        })
    }

    pub fn destinations(&self) -> &Destinations {
        &self.destinations
    }

    pub fn best_solution(&self) -> Option<&PlanningResult> {
        self.best.as_ref()
    }

    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// Single planning iteration (skeleton — returns NotImplemented until Phase 3+).
    pub fn step(&mut self) -> Result<StepResult> {
        if self.finished {
            return Ok(StepResult {
                status: StepStatus::Finished,
                iteration: self.iteration,
                best_cost: self.best.as_ref().map(|b| b.cost),
            });
        }

        self.iteration += 1;
        if self.iteration >= self.config.general.max_iter {
            self.finished = true;
        }

        Err(PlannerError::NotImplemented("ImomdRrtStar::step"))
    }

    pub fn run_until(&mut self, deadline: Instant) -> Result<PlanningResult> {
        while Instant::now() < deadline && !self.finished {
            match self.step() {
                Ok(_) => {}
                Err(PlannerError::NotImplemented(_)) => {
                    self.finished = true;
                    break;
                }
                Err(e) => return Err(e),
            }
        }
        self.best.clone().ok_or(PlannerError::NotImplemented(
            "no solution found yet",
        ))
    }

    pub fn run_for(&mut self, duration: Duration) -> Result<PlanningResult> {
        self.run_until(Instant::now() + duration)
    }

    /// Build initial tree layers (one per destination).
    pub fn init_trees(&self) -> Vec<RrtTree> {
        self.destinations
            .all_nodes()
            .iter()
            .enumerate()
            .map(|(i, &root)| RrtTree::new(i as i32, root))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::*;
    use crate::map::FakeMapLoader;
    use crate::map::MapLoader;

    fn test_config() -> AlgorithmConfig {
        AlgorithmConfig::from_yaml_str(
            r#"
general: { system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: 10, max_time: 1 }
rrt_params: { goal_bias: 1.0, random_seed: 0 }
destinations: { source_id: 0, objective_ids: [1], target_id: 2 }
map: { type: -1, path: "", name: "" }
rtsp_settings: { shortcut: 1, swapping: 1, genetic: 1, ga: { random_seed: 0, mutation_iter: 10, population: 10, generation: 1 } }
"#,
        )
        .unwrap()
    }

    #[test]
    fn planner_init_trees_count() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1],
            target: 2,
        };
        let planner = ImomdRrtStar::new(graph, dest, test_config()).unwrap();
        assert_eq!(planner.init_trees().len(), 3);
    }

    #[test]
    fn planner_rejects_invalid_node() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![99],
            target: 2,
        };
        assert!(ImomdRrtStar::new(graph, dest, test_config()).is_err());
    }
}
