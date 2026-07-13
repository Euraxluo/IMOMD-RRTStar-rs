use crate::error::{PlannerError, Result};
use crate::geo::haversine_distance;
use crate::types::{Location, NodeId};
use rustc_hash::FxHashMap;

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
        let nodes = vec![
            Location::new(0, 0.0, 0.0),
            Location::new(1, 1.0, 0.0),
        ];
        let mut e0 = FxHashMap::default();
        e0.insert(1, 100.0);
        let mut e1 = FxHashMap::default();
        e1.insert(0, 100.0);
        let g = AdjacencyGraph::new(nodes, vec![e0, e1]).unwrap();
        assert_eq!(g.neighbors(0), vec![(1, 100.0)]);
    }
}
