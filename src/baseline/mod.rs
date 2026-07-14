mod ana_star;
mod bi_astar;
mod common;

pub use ana_star::AnaStar;
pub use bi_astar::BiAstar;

use crate::error::Result;
use crate::types::PlanningResult;

/// Baseline planner trait shared by Bi-A* and ANA*.
pub trait BaselinePlanner {
    fn find_shortest_path(&mut self) -> Result<PlanningResult>;
}
