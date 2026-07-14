use std::sync::Arc;
use std::time::Duration;

use crate::error::Result;
use crate::graph::AdjacencyGraph;
use crate::navigation::events::PlanUpdate;
use crate::types::{Destinations, GraphUpdateStats, NodeId, PlanningResult};

/// Pluggable realtime planner backend.
///
/// IMOMD-RRT* is the first implementation; LPA* / D* Lite / CCH adapters can
/// implement the same contract and drop into [`crate::navigation::NavigationSession`].
pub trait PlannerPlugin: Send + Sync {
    fn id(&self) -> &'static str;

    fn reset(&mut self, graph: Arc<AdjacencyGraph>, destinations: Destinations) -> Result<()>;

    fn on_graph_changed(&mut self, graph: Arc<AdjacencyGraph>) -> Result<GraphUpdateStats>;

    /// Reseed planning from a new ego node while keeping remaining goals.
    fn on_ego_moved(&mut self, ego_node: NodeId, remaining: Destinations) -> Result<()>;

    /// Spend a time budget and return zero or more streamable updates.
    fn continue_search(&mut self, budget: Duration) -> Result<Vec<PlanUpdate>>;

    fn best(&self) -> Option<&PlanningResult>;

    fn is_finished(&self) -> bool;
}
