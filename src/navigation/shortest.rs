//! Dijkstra helpers on [`RoadGraph`] for greedy / exact race lanes.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::graph::RoadGraph;
use crate::types::NodeId;

#[derive(Clone, Copy)]
struct State {
    cost: f64,
    node: NodeId,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.node == other.node
    }
}

impl Eq for State {}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap via reverse cost ordering.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.node.cmp(&other.node))
    }
}

/// Shortest path cost and node sequence from `source` to `target`.
/// Returns `None` when disconnected or non-finite.
pub fn dijkstra_path(
    graph: &dyn RoadGraph,
    source: NodeId,
    target: NodeId,
) -> Option<(f64, Vec<NodeId>)> {
    if source == target {
        return Some((0.0, vec![source]));
    }
    if source >= graph.node_count() || target >= graph.node_count() {
        return None;
    }

    let n = graph.node_count();
    let mut dist = vec![f64::INFINITY; n];
    let mut parent: Vec<Option<NodeId>> = vec![None; n];
    let mut heap = BinaryHeap::new();
    dist[source] = 0.0;
    heap.push(State {
        cost: 0.0,
        node: source,
    });

    while let Some(State { cost, node }) = heap.pop() {
        if cost > dist[node] {
            continue;
        }
        if node == target {
            break;
        }
        for (next, weight) in graph.neighbors(node) {
            if !weight.is_finite() || weight < 0.0 {
                continue;
            }
            let cand = cost + weight;
            if cand < dist[next] {
                dist[next] = cand;
                parent[next] = Some(node);
                heap.push(State {
                    cost: cand,
                    node: next,
                });
            }
        }
    }

    if !dist[target].is_finite() {
        return None;
    }

    let mut path = Vec::new();
    let mut cur = Some(target);
    while let Some(node) = cur {
        path.push(node);
        cur = parent[node];
    }
    path.reverse();
    if path.first().copied() != Some(source) {
        return None;
    }
    Some((dist[target], path))
}

/// Concatenate leg paths, dropping duplicate joint nodes.
pub fn stitch_legs(legs: &[Vec<NodeId>]) -> Vec<NodeId> {
    let mut out = Vec::new();
    for leg in legs {
        if leg.is_empty() {
            continue;
        }
        if out.is_empty() {
            out.extend_from_slice(leg);
        } else {
            out.extend_from_slice(&leg[1..]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::AdjacencyGraph;
    use crate::types::Location;
    use rustc_hash::FxHashMap;
    use std::sync::Arc;

    fn line_graph() -> Arc<AdjacencyGraph> {
        let nodes = vec![
            Location::new(0, 0.0, 0.0),
            Location::new(1, 0.0, 1.0),
            Location::new(2, 0.0, 2.0),
        ];
        let mut edges = vec![FxHashMap::default(); 3];
        edges[0].insert(1, 10.0);
        edges[1].insert(0, 10.0);
        edges[1].insert(2, 5.0);
        edges[2].insert(1, 5.0);
        Arc::new(AdjacencyGraph::new(nodes, edges).unwrap())
    }

    #[test]
    fn dijkstra_finds_detour() {
        let g = line_graph();
        let (cost, path) = dijkstra_path(g.as_ref(), 0, 2).unwrap();
        assert!((cost - 15.0).abs() < 1e-9);
        assert_eq!(path, vec![0, 1, 2]);
    }
}
