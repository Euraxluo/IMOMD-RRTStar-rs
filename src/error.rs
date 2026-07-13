use crate::types::NodeId;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum PlannerError {
    #[error("node {0} not found in graph")]
    NodeNotFound(NodeId),

    #[error("graph is disconnected between {0} and {1}")]
    Disconnected(NodeId, NodeId),

    #[error("invalid config: {0}")]
    Config(String),

    #[error("planning timeout after {0}s")]
    Timeout(f64),

    #[error("map load error: {0}")]
    MapLoad(String),

    #[error("not implemented: {0}")]
    NotImplemented(&'static str),
}

pub type Result<T> = std::result::Result<T, PlannerError>;
