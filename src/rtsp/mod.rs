pub mod greedy;
pub mod eci_gen;

pub use eci_gen::EciGenSolver;
pub use greedy::GreedyTspSolver;

use crate::config::RtspSettings;
use crate::error::Result;

/// Solve the relaxed TSP given a symmetric distance matrix between destination trees.
pub trait RtspSolver: Send + Sync {
    fn solve(&self, distance_matrix: &[Vec<f64>], settings: &RtspSettings) -> Result<Vec<usize>>;
}
