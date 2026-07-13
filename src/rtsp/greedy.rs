use crate::config::RtspSettings;
use crate::error::{PlannerError, Result};
use crate::rtsp::RtspSolver;

/// Greedy cheapest-insertion TSP solver (maps to C++ `greedy_tsp.h`).
pub struct GreedyTspSolver;

impl RtspSolver for GreedyTspSolver {
    fn solve(&self, _distance_matrix: &[Vec<f64>], _settings: &RtspSettings) -> Result<Vec<usize>> {
        Err(PlannerError::NotImplemented("GreedyTspSolver::solve"))
    }
}
