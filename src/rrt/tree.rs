use rustc_hash::{FxHashMap, FxHashSet};

use crate::types::NodeId;

/// RRT* tree rooted at a destination node (maps to C++ `tree_t`).
#[derive(Debug, Clone)]
pub struct RrtTree {
    pub id: i32,
    pub root: NodeId,
    pub parent: FxHashMap<NodeId, NodeId>,
    pub children: FxHashMap<NodeId, FxHashSet<NodeId>>,
    pub cost: FxHashMap<NodeId, f64>,
    pub expandables: FxHashSet<NodeId>,
    pub is_done: bool,
}

impl RrtTree {
    pub fn new(id: i32, root: NodeId) -> Self {
        let mut parent = FxHashMap::default();
        parent.insert(root, root);
        let mut children = FxHashMap::default();
        children.insert(root, FxHashSet::default());
        let mut cost = FxHashMap::default();
        cost.insert(root, 0.0);
        Self {
            id,
            root,
            parent,
            children,
            cost,
            expandables: FxHashSet::default(),
            is_done: false,
        }
    }

    pub fn is_visited(&self, node: NodeId) -> bool {
        self.parent.contains_key(&node)
    }

    /// Propagate cost update from `rewired_node` to all descendants.
    /// Returns visited nodes in BFS order (rewired node first), matching C++ `updateCost`.
    pub fn update_cost(&mut self, rewired_node: NodeId, new_cost: f64) -> Vec<NodeId> {
        let old_cost = *self.cost.get(&rewired_node).unwrap_or(&0.0);
        let delta = new_cost - old_cost;

        let mut updated = Vec::new();
        let mut queue = std::collections::VecDeque::from([rewired_node]);
        while let Some(parent) = queue.pop_front() {
            updated.push(parent);
            if let Some(kids) = self.children.get(&parent) {
                for &child in kids {
                    queue.push_back(child);
                }
            }
        }
        for &node in &updated {
            if let Some(c) = self.cost.get_mut(&node) {
                *c += delta;
            }
        }
        self.cost.insert(rewired_node, new_cost);
        updated
    }

    pub fn add_child(&mut self, parent: NodeId, child: NodeId, edge_cost: f64) {
        self.parent.insert(child, parent);
        self.children.entry(parent).or_default().insert(child);
        let parent_cost = *self.cost.get(&parent).unwrap_or(&0.0);
        self.cost.insert(child, parent_cost + edge_cost);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_tree_root_is_visited() {
        let tree = RrtTree::new(0, 5);
        assert!(tree.is_visited(5));
        assert!(!tree.is_visited(99));
    }

    #[test]
    fn update_cost_propagates_to_children() {
        let mut tree = RrtTree::new(0, 0);
        tree.add_child(0, 1, 10.0);
        tree.add_child(1, 2, 5.0);
        tree.update_cost(1, 8.0);
        assert!((tree.cost[&2] - 13.0).abs() < 1e-9);
    }
}
