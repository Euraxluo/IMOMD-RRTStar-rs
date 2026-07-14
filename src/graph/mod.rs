use crate::error::{PlannerError, Result};
use crate::geo::haversine_distance;
use crate::types::{Location, NodeId};
use rustc_hash::FxHashMap;

mod traffic;

pub use traffic::{EdgeView, NodeView, TrafficGraph, TrafficLevel, TrafficView};

/// Road network abstraction matching the C++ graph interface.
pub trait RoadGraph: Send + Sync {
    fn node_count(&self) -> usize;
    fn location(&self, id: NodeId) -> Option<&Location>;
    fn neighbors(&self, id: NodeId) -> Vec<(NodeId, f64)>;
    fn edge_weight(&self, from: NodeId, to: NodeId) -> Option<f64>;
    fn haversine(&self, a: NodeId, b: NodeId) -> Option<f64>;
}

/// Adjacency-list road graph: `Vec<Location>` nodes + `Vec<FxHashMap<NodeId, f64>>` edges.
#[derive(Debug, Clone)]
pub struct AdjacencyGraph {
    nodes: Vec<Location>,
    edges: Vec<FxHashMap<NodeId, f64>>,
}

impl AdjacencyGraph {
    pub fn new(nodes: Vec<Location>, edges: Vec<FxHashMap<NodeId, f64>>) -> Result<Self> {
        if nodes.len() != edges.len() {
            return Err(PlannerError::MapLoad(format!(
                "node count {} != edge list count {}",
                nodes.len(),
                edges.len()
            )));
        }
        for (index, node) in nodes.iter().enumerate() {
            if node.id != index {
                return Err(PlannerError::MapLoad(format!(
                    "node id {} does not match adjacency index {index}",
                    node.id
                )));
            }
            if !node.latitude.is_finite()
                || !node.longitude.is_finite()
                || !(-90.0..=90.0).contains(&node.latitude)
                || !(-180.0..=180.0).contains(&node.longitude)
            {
                return Err(PlannerError::MapLoad(format!(
                    "node {index} has invalid coordinates ({}, {})",
                    node.latitude, node.longitude
                )));
            }
        }
        for (from, neighbors) in edges.iter().enumerate() {
            for (&to, &weight) in neighbors {
                if to >= nodes.len() {
                    return Err(PlannerError::MapLoad(format!(
                        "edge ({from}, {to}) references unknown node"
                    )));
                }
                if !weight.is_finite() || weight < 0.0 {
                    return Err(PlannerError::MapLoad(format!(
                        "edge ({from}, {to}) has invalid weight {weight}"
                    )));
                }
            }
        }
        Ok(Self { nodes, edges })
    }

    pub fn nodes(&self) -> &[Location] {
        &self.nodes
    }

    pub fn edges(&self) -> &[FxHashMap<NodeId, f64>] {
        &self.edges
    }
}

impl RoadGraph for AdjacencyGraph {
    fn node_count(&self) -> usize {
        self.nodes.len()
    }

    fn location(&self, id: NodeId) -> Option<&Location> {
        self.nodes.get(id)
    }

    fn neighbors(&self, id: NodeId) -> Vec<(NodeId, f64)> {
        self.edges
            .get(id)
            .map(|m| m.iter().map(|(&n, &w)| (n, w)).collect())
            .unwrap_or_default()
    }

    fn edge_weight(&self, from: NodeId, to: NodeId) -> Option<f64> {
        self.edges.get(from)?.get(&to).copied()
    }

    fn haversine(&self, a: NodeId, b: NodeId) -> Option<f64> {
        let loc_a = self.location(a)?;
        let loc_b = self.location(b)?;
        Some(haversine_distance(
            loc_a.latitude,
            loc_a.longitude,
            loc_b.latitude,
            loc_b.longitude,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adjacency_graph_neighbors() {
        let nodes = vec![Location::new(0, 0.0, 0.0), Location::new(1, 1.0, 0.0)];
        let mut e0 = FxHashMap::default();
        e0.insert(1, 100.0);
        let mut e1 = FxHashMap::default();
        e1.insert(0, 100.0);
        let g = AdjacencyGraph::new(nodes, vec![e0, e1]).unwrap();
        assert_eq!(g.neighbors(0), vec![(1, 100.0)]);
    }

    #[test]
    fn adjacency_graph_rejects_invalid_identity_coordinates_and_edges() {
        let empty = FxHashMap::default();
        assert!(
            AdjacencyGraph::new(vec![Location::new(1, 0.0, 0.0)], vec![empty.clone()]).is_err()
        );
        assert!(
            AdjacencyGraph::new(vec![Location::new(0, f64::NAN, 0.0)], vec![empty.clone()])
                .is_err()
        );

        let mut invalid_edge = FxHashMap::default();
        invalid_edge.insert(1, 1.0);
        assert!(AdjacencyGraph::new(vec![Location::new(0, 0.0, 0.0)], vec![invalid_edge]).is_err());
    }
}
