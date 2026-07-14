use std::f64;

use rustc_hash::FxHashMap;

use crate::error::Result;
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::types::{Location, NodeId};

fn canonical_edge(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

/// V2X-style traffic level applied as travel-time multipliers on base edge weights.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrafficLevel {
    /// Normal flow (×1.0)
    Free,
    /// Congested / slow (×2.5)
    Slow,
    /// Heavy jam (×5.0)
    Jam,
    /// Road closed
    Blocked,
}

impl TrafficLevel {
    pub fn multiplier(self) -> f64 {
        match self {
            Self::Free => 1.0,
            Self::Slow => 2.5,
            Self::Jam => 5.0,
            Self::Blocked => f64::INFINITY,
        }
    }

    pub fn from_label(label: &str) -> Option<Self> {
        match label.to_ascii_lowercase().as_str() {
            "free" | "clear" | "normal" => Some(Self::Free),
            "slow" | "congested" => Some(Self::Slow),
            "jam" | "blocked_slow" => Some(Self::Jam),
            "blocked" | "closed" => Some(Self::Blocked),
            _ => None,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Free => "free",
            Self::Slow => "slow",
            Self::Jam => "jam",
            Self::Blocked => "blocked",
        }
    }
}

/// Mutable traffic overlay on top of a static road network.
#[derive(Debug, Clone)]
pub struct TrafficGraph {
    base: AdjacencyGraph,
    edge_levels: FxHashMap<(NodeId, NodeId), TrafficLevel>,
}

impl TrafficGraph {
    pub fn new(base: AdjacencyGraph) -> Self {
        Self {
            base,
            edge_levels: FxHashMap::default(),
        }
    }

    pub fn base(&self) -> &AdjacencyGraph {
        &self.base
    }

    pub fn node_count(&self) -> usize {
        self.base.node_count()
    }

    pub fn set_edge_level(&mut self, from: NodeId, to: NodeId, level: TrafficLevel) {
        let key = canonical_edge(from, to);
        if level == TrafficLevel::Free {
            self.edge_levels.remove(&key);
        } else {
            self.edge_levels.insert(key, level);
        }
    }

    /// Apply traffic to all edges incident to any node in `zone_nodes`.
    pub fn set_zone_level(&mut self, zone_nodes: &[NodeId], level: TrafficLevel) {
        let zone: FxHashMap<NodeId, ()> = zone_nodes.iter().copied().map(|n| (n, ())).collect();
        for node in 0..self.base.node_count() {
            for (neighbor, _) in self.base.neighbors(node) {
                if zone.contains_key(&node) || zone.contains_key(&neighbor) {
                    self.set_edge_level(node, neighbor, level);
                }
            }
        }
    }

    pub fn clear_traffic(&mut self) {
        self.edge_levels.clear();
    }

    pub fn edge_level(&self, from: NodeId, to: NodeId) -> TrafficLevel {
        self.edge_levels
            .get(&canonical_edge(from, to))
            .copied()
            .unwrap_or(TrafficLevel::Free)
    }

    /// Build a weighted graph snapshot for the planner under current traffic.
    pub fn materialize(&self) -> Result<AdjacencyGraph> {
        let nodes: Vec<Location> = self.base.nodes().to_vec();
        let mut edges = vec![FxHashMap::default(); nodes.len()];

        for (u, base_edges) in self.base.edges().iter().enumerate() {
            for (&v, &base_w) in base_edges {
                if u >= v {
                    continue;
                }
                let level = self.edge_level(u, v);
                let factor = level.multiplier();
                if !factor.is_finite() {
                    continue;
                }
                let weight = base_w * factor;
                edges[u].insert(v, weight);
                edges[v].insert(u, weight);
            }
        }

        AdjacencyGraph::new(nodes, edges)
    }

    /// Export nodes/edges for visualization APIs.
    pub fn export_view(&self) -> TrafficView {
        let nodes: Vec<NodeView> = self
            .base
            .nodes()
            .iter()
            .map(|loc| NodeView {
                id: loc.id,
                latitude: loc.latitude,
                longitude: loc.longitude,
            })
            .collect();

        let mut edges = Vec::new();
        for (u, base_edges) in self.base.edges().iter().enumerate() {
            for (&v, &base_w) in base_edges {
                if u >= v {
                    continue;
                }
                let level = self.edge_level(u, v);
                let factor = level.multiplier();
                let effective_weight = if factor.is_finite() {
                    base_w * factor
                } else {
                    f64::INFINITY
                };
                edges.push(EdgeView {
                    from: u,
                    to: v,
                    base_weight: base_w,
                    effective_weight,
                    level: level.label().to_string(),
                });
            }
        }

        TrafficView { nodes, edges }
    }
}

#[derive(Debug, Clone)]
pub struct NodeView {
    pub id: NodeId,
    pub latitude: f64,
    pub longitude: f64,
}

#[derive(Debug, Clone)]
pub struct EdgeView {
    pub from: NodeId,
    pub to: NodeId,
    pub base_weight: f64,
    pub effective_weight: f64,
    pub level: String,
}

#[derive(Debug, Clone)]
pub struct TrafficView {
    pub nodes: Vec<NodeView>,
    pub edges: Vec<EdgeView>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::haversine_distance;
    use crate::graph::RoadGraph;

    #[test]
    fn jam_increases_edge_weight() {
        let nodes = vec![Location::new(0, 0.0, 0.0), Location::new(1, 1.0, 0.0)];
        let w = haversine_distance(0.0, 0.0, 1.0, 0.0);
        let mut e0 = FxHashMap::default();
        e0.insert(1, w);
        let mut e1 = FxHashMap::default();
        e1.insert(0, w);
        let base = AdjacencyGraph::new(nodes, vec![e0, e1]).unwrap();

        let mut traffic = TrafficGraph::new(base);
        traffic.set_edge_level(0, 1, TrafficLevel::Jam);
        let live = traffic.materialize().unwrap();
        assert!(live.edge_weight(0, 1).unwrap() > w * 4.0);
    }

    #[test]
    fn blocked_edge_removed_from_materialized_graph() {
        let nodes = vec![Location::new(0, 0.0, 0.0), Location::new(1, 1.0, 0.0)];
        let w = 100.0;
        let mut e0 = FxHashMap::default();
        e0.insert(1, w);
        let mut e1 = FxHashMap::default();
        e1.insert(0, w);
        let base = AdjacencyGraph::new(nodes, vec![e0, e1]).unwrap();

        let mut traffic = TrafficGraph::new(base);
        traffic.set_edge_level(0, 1, TrafficLevel::Blocked);
        let live = traffic.materialize().unwrap();
        assert!(live.neighbors(0).is_empty());
    }
}
