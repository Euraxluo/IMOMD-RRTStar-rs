pub mod planner;
pub mod tree;

pub use planner::ImomdRrtStar;
pub use tree::RrtTree;

use std::time::Instant;

use crate::error::Result;
use crate::types::{PlanningResult, StepResult};

/// Common contract for planners that can be paused and resumed while retaining
/// their search state and best solution.
pub trait AnytimePlanner {
    fn step(&mut self) -> Result<StepResult>;
    fn best_solution(&self) -> Option<&PlanningResult>;
    fn is_finished(&self) -> bool;
    fn run_until(&mut self, deadline: Instant) -> Result<PlanningResult>;
}
