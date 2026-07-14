use serde::{Deserialize, Serialize};

use crate::types::{GraphUpdateStats, NodeId, PlanningResult};

/// Why a plan update was emitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UpdateReason {
    Expanded,
    Connected,
    Improved,
    Finished,
    EgoReseed,
    TrafficWarmStart,
    Fresh,
    Resume,
    /// Greedy NN/CI tour lane in the solver race.
    GreedyInit,
    /// Exact Dijkstra + TSP lane (optimal on current graph when eligible).
    ExactOptimal,
}

/// Incremental navigation update for streaming clients.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanUpdate {
    pub sequence: u64,
    pub reason: UpdateReason,
    pub path: Option<Vec<NodeId>>,
    pub cost: Option<f64>,
    pub visit_order: Option<Vec<usize>>,
    pub explored_nodes: Option<usize>,
    pub replan_mode: String,
    pub tree_update: Option<GraphUpdateStats>,
    pub ego_node: Option<NodeId>,
    pub algorithm_id: String,
}

impl PlanUpdate {
    pub fn from_best(
        sequence: u64,
        reason: UpdateReason,
        best: &PlanningResult,
        replan_mode: impl Into<String>,
        algorithm_id: impl Into<String>,
        tree_update: Option<GraphUpdateStats>,
        ego_node: Option<NodeId>,
    ) -> Self {
        Self {
            sequence,
            reason,
            path: Some(best.path.clone()),
            cost: Some(best.cost),
            visit_order: Some(best.visit_order.clone()),
            explored_nodes: Some(best.explored_nodes),
            replan_mode: replan_mode.into(),
            tree_update,
            ego_node,
            algorithm_id: algorithm_id.into(),
        }
    }

    pub fn marker(
        sequence: u64,
        reason: UpdateReason,
        replan_mode: impl Into<String>,
        algorithm_id: impl Into<String>,
        tree_update: Option<GraphUpdateStats>,
        ego_node: Option<NodeId>,
    ) -> Self {
        Self {
            sequence,
            reason,
            path: None,
            cost: None,
            visit_order: None,
            explored_nodes: None,
            replan_mode: replan_mode.into(),
            tree_update,
            ego_node,
            algorithm_id: algorithm_id.into(),
        }
    }
}

/// Session-level inputs that drive replanning.
#[derive(Debug, Clone)]
pub enum DomainEvent {
    /// Replace destinations (source / objectives / target).
    DestinationsSet {
        source: NodeId,
        objectives: Vec<NodeId>,
        target: NodeId,
    },
    /// Road costs changed; caller supplies a newly materialized graph.
    TrafficChanged,
    /// Ego / vehicle snapped to a graph node.
    EgoMoved { ego_node: NodeId },
    /// Spend more search budget without changing inputs.
    ContinueSearch,
}
