use crate::config::AlgorithmConfig;
use crate::error::Result;
use crate::graph::AdjacencyGraph;
use crate::types::{Destinations, PlanningResult};
use std::sync::Arc;

use super::BaselinePlanner;

/// ANA* baseline (maps to C++ `ANAStar`).
pub struct AnaStar {
    _graph: Arc<AdjacencyGraph>,
    _destinations: Destinations,
    _config: AlgorithmConfig,
}

impl AnaStar {
    pub fn new(
        graph: Arc<AdjacencyGraph>,
        destinations: Destinations,
        config: AlgorithmConfig,
    ) -> Self {
        Self {
            _graph: graph,
            _destinations: destinations,
            _config: config,
        }
    }
}

impl BaselinePlanner for AnaStar {
    fn find_shortest_path(&mut self) -> Result<PlanningResult> {
        Err(crate::error::PlannerError::NotImplemented("AnaStar"))
    }
}
