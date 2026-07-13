use serde::{Deserialize, Serialize};

pub type NodeId = usize;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Location {
    pub id: NodeId,
    pub latitude: f64,
    pub longitude: f64,
}

impl Location {
    pub fn new(id: NodeId, latitude: f64, longitude: f64) -> Self {
        Self {
            id,
            latitude,
            longitude,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Destinations {
    pub source: NodeId,
    pub objectives: Vec<NodeId>,
    pub target: NodeId,
}

impl Destinations {
    pub fn all_nodes(&self) -> Vec<NodeId> {
        let mut nodes = vec![self.source];
        nodes.extend_from_slice(&self.objectives);
        nodes.push(self.target);
        nodes
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PlanningResult {
    pub path: Vec<NodeId>,
    pub visit_order: Vec<usize>,
    pub cost: f64,
    pub explored_nodes: usize,
    pub elapsed_secs: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum PlannerSystem {
    Imomd = 0,
    BiAstar = 1,
    AnaStar = 2,
}

impl PlannerSystem {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Imomd),
            1 => Some(Self::BiAstar),
            2 => Some(Self::AnaStar),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepStatus {
    Expanded,
    Connected,
    PathImproved,
    Finished,
}

#[derive(Debug, Clone)]
pub struct StepResult {
    pub status: StepStatus,
    pub iteration: usize,
    pub best_cost: Option<f64>,
}
