pub mod eci_gen;
pub mod greedy;

pub use eci_gen::EciGenSolver;
pub use greedy::solve_brute_force;

/// Symmetric matrix of shortest known distances between destination trees.
/// Unconnected pairs are `f64::INFINITY`; the diagonal is 0.
pub type DistanceMatrix = Vec<Vec<f64>>;

/// Result of solving the relaxed TSP over destination trees.
#[derive(Debug, Clone, PartialEq)]
pub struct RtspSolution {
    pub cost: f64,
    pub visit_order: Vec<usize>,
}

/// Replaceable relaxed-TSP solver contract used by the planning framework.
pub trait RtspSolver {
    fn solve(
        &mut self,
        distance_matrix: &DistanceMatrix,
        source_id: usize,
        target_id: usize,
    ) -> RtspSolution;
}
