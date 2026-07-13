use crate::config::RtspSettings;
use crate::error::{PlannerError, Result};
use crate::rtsp::{GreedyTspSolver, RtspSolver};

/// Enhanced Cheapest Insertion + Genetic Algorithm RTSP solver.
/// Maps to C++ `eci_gen_tsp_solver.h`.
pub struct EciGenSolver {
    greedy: GreedyTspSolver,
}

impl Default for EciGenSolver {
    fn default() -> Self {
        Self {
            greedy: GreedyTspSolver,
        }
    }
}

impl EciGenSolver {
    pub fn new() -> Self {
        Self::default()
    }
}

impl RtspSolver for EciGenSolver {
    fn solve(&self, distance_matrix: &[Vec<f64>], settings: &RtspSettings) -> Result<Vec<usize>> {
        let initial = self.greedy.solve(distance_matrix, settings)?;
        if settings.genetic == 0 {
            return Ok(initial);
        }
        Err(PlannerError::NotImplemented("EciGenSolver genetic improvement"))
    }
}
