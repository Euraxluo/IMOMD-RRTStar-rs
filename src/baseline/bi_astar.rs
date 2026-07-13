use crate::config::AlgorithmConfig;
use crate::error::Result;
use crate::graph::AdjacencyGraph;
use crate::types::{Destinations, PlanningResult};
use std::sync::Arc;

use super::BaselinePlanner;

/// Bidirectional A* baseline (maps to C++ `BiAstar`).
pub struct BiAstar {
    _graph: Arc<AdjacencyGraph>,
    _destinations: Destinations,
    _config: AlgorithmConfig,
}

impl BiAstar {
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

impl BaselinePlanner for BiAstar {
    fn find_shortest_path(&mut self) -> Result<PlanningResult> {
        Err(crate::error::PlannerError::NotImplemented("BiAstar"))
    }
}
