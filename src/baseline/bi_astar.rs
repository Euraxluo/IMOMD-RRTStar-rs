use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::sync::Arc;

use rand::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::config::AlgorithmConfig;
use crate::error::Result;
use crate::experiment::ExperimentRecord;
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::types::{Destinations, NodeId, PlanningResult};

use super::common::{pair_tree_idx, BaselineCore, OrderedFloat, PriorityState};
use super::BaselinePlanner;

struct BiAstarTree {
    start_id: i32,
    goal_id: i32,
    start_root: NodeId,
    goal_root: NodeId,
    parent_forward: FxHashMap<NodeId, NodeId>,
    parent_backward: FxHashMap<NodeId, NodeId>,
    cost_forward: FxHashMap<NodeId, f64>,
    cost_backward: FxHashMap<NodeId, f64>,
    open_forward: BinaryHeap<Reverse<PriorityState>>,
    open_backward: BinaryHeap<Reverse<PriorityState>>,
}

impl BiAstarTree {
    fn new(start_id: i32, goal_id: i32, start_root: NodeId, goal_root: NodeId) -> Self {
        let mut parent_forward = FxHashMap::default();
        parent_forward.insert(start_root, start_root);
        let mut parent_backward = FxHashMap::default();
        parent_backward.insert(goal_root, goal_root);
        let mut cost_forward = FxHashMap::default();
        cost_forward.insert(start_root, 0.0);
        let mut cost_backward = FxHashMap::default();
        cost_backward.insert(goal_root, 0.0);

        let mut open_forward = BinaryHeap::new();
        open_forward.push(Reverse(PriorityState {
            priority: OrderedFloat::new(0.0),
            node: start_root,
        }));
        let mut open_backward = BinaryHeap::new();
        open_backward.push(Reverse(PriorityState {
            priority: OrderedFloat::new(0.0),
            node: goal_root,
        }));

        Self {
            start_id,
            goal_id,
            start_root,
            goal_root,
            parent_forward,
            parent_backward,
            cost_forward,
            cost_backward,
            open_forward,
            open_backward,
        }
    }
}

/// Bidirectional A* baseline (maps to C++ `BiAstar`).
pub struct BiAstar {
    core: BaselineCore,
    tree_layers: Vec<BiAstarTree>,
    unexplored_tree: FxHashSet<usize>,
}

impl BiAstar {
    pub fn new(
        graph: Arc<AdjacencyGraph>,
        destinations: Destinations,
        config: AlgorithmConfig,
    ) -> Result<Self> {
        let mut core = BaselineCore::new(
            graph,
            config,
            destinations.source,
            destinations.objectives,
            destinations.target,
        )?;

        let dest_count = core.destinations.len();
        let mut tree_layers = Vec::new();
        let mut unexplored_tree = FxHashSet::default();

        for i in 0..dest_count {
            core.disjoint_set_children.insert(i as i32, Vec::new());
            for j in (i + 1)..dest_count {
                let idx = pair_tree_idx(i as i32, j as i32);
                tree_layers.push(BiAstarTree::new(
                    i as i32,
                    j as i32,
                    core.destinations[i],
                    core.destinations[j],
                ));
                unexplored_tree.insert(idx);
            }
        }

        Ok(Self {
            core,
            tree_layers,
            unexplored_tree,
        })
    }

    pub fn experiment_records(&self) -> &[ExperimentRecord] {
        self.core.experiment_records()
    }

    fn expand_tree_layers(&mut self) {
        if self.unexplored_tree.is_empty() {
            return;
        }
        let tree_id = *self
            .unexplored_tree
            .iter()
            .nth(self.core.rng.gen_range(0..self.unexplored_tree.len()))
            .unwrap();
        self.expand_tree(tree_id);
        self.unexplored_tree.remove(&tree_id);
    }

    fn expand_tree(&mut self, tree_id: usize) {
        let tree = &mut self.tree_layers[tree_id];
        let graph = Arc::clone(&self.core.graph);
        let mut connection_node = None;

        while !tree.open_forward.is_empty() && !tree.open_backward.is_empty() {
            let Reverse(PriorityState { node: state, .. }) = tree.open_forward.pop().unwrap();
            if tree.parent_backward.contains_key(&state) {
                connection_node = Some(state);
                break;
            }

            for (neighbor, weight) in graph.neighbors(state) {
                let new_cost = tree.cost_forward[&state] + weight;
                let entry = tree.cost_forward.entry(neighbor).or_insert(f64::INFINITY);
                if new_cost < *entry {
                    *entry = new_cost;
                    tree.parent_forward.insert(neighbor, state);
                    let f = new_cost + self.core.heuristic(neighbor, tree.goal_root);
                    tree.open_forward.push(Reverse(PriorityState {
                        priority: OrderedFloat::new(f),
                        node: neighbor,
                    }));
                }
            }

            let Reverse(PriorityState { node: state, .. }) = tree.open_backward.pop().unwrap();
            if tree.parent_forward.contains_key(&state) {
                connection_node = Some(state);
                break;
            }

            for (neighbor, weight) in graph.neighbors(state) {
                let new_cost = tree.cost_backward[&state] + weight;
                let entry = tree.cost_backward.entry(neighbor).or_insert(f64::INFINITY);
                if new_cost < *entry {
                    *entry = new_cost;
                    tree.parent_backward.insert(neighbor, state);
                    let f = new_cost + self.core.heuristic(neighbor, tree.start_root);
                    tree.open_backward.push(Reverse(PriorityState {
                        priority: OrderedFloat::new(f),
                        node: neighbor,
                    }));
                }
            }
        }

        let Some(connection_node) = connection_node else {
            return;
        };

        let distance = tree.cost_forward[&connection_node] + tree.cost_backward[&connection_node];
        let si = tree.start_id as usize;
        let gi = tree.goal_id as usize;

        if distance < self.core.distance_matrix[si][gi] - 0.1 {
            self.core.distance_matrix[si][gi] = distance;
            self.core.distance_matrix[gi][si] = distance;
            self.core.connection_node_matrix[si][gi] = Some(connection_node);
            self.core.connection_node_matrix[gi][si] = Some(connection_node);
            self.core.is_distance_matrix_updated = true;
        }

        self.core.connect_two_tree(tree.start_id, tree.goal_id);
    }

    fn update_path(&mut self) {
        self.core.shortest_path.clear();
        for window in self.core.sequence_rtsp.windows(2) {
            let start_id = window[0] as i32;
            let goal_id = window[1] as i32;
            let tree_idx = pair_tree_idx(start_id, goal_id);
            let connection_node = self.core.connection_node_matrix[start_id as usize]
                [goal_id as usize]
                .unwrap_or(self.core.destinations[start_id as usize]);

            let tree = &self.tree_layers[tree_idx];
            let mut node = connection_node;
            let mut tmp_path = Vec::new();

            if tree.start_id == start_id {
                while node != tree.start_root {
                    node = tree.parent_forward[&node];
                    tmp_path.push(node);
                }
                for &n in tmp_path.iter().rev() {
                    self.core.shortest_path.push(n);
                }
                node = connection_node;
                while node != tree.goal_root {
                    self.core.shortest_path.push(node);
                    node = tree.parent_backward[&node];
                }
            } else {
                while node != tree.goal_root {
                    node = tree.parent_backward[&node];
                    tmp_path.push(node);
                }
                for &n in tmp_path.iter().rev() {
                    self.core.shortest_path.push(n);
                }
                node = connection_node;
                while node != tree.start_root {
                    self.core.shortest_path.push(node);
                    node = tree.parent_forward[&node];
                }
            }
        }
        self.core
            .shortest_path
            .push(*self.core.destinations.last().unwrap());
    }

    fn explored_nodes(&self) -> usize {
        self.tree_layers
            .iter()
            .map(|t| t.parent_forward.len() + t.parent_backward.len())
            .sum()
    }
}

impl BaselinePlanner for BiAstar {
    fn find_shortest_path(&mut self) -> Result<PlanningResult> {
        while !self.core.timed_out() && !self.unexplored_tree.is_empty() {
            self.expand_tree_layers();
            if self.core.solve_rtsp(false) {
                self.update_path();
                let explored_nodes = self.explored_nodes();
                self.core.log_data(explored_nodes);
            }
            self.core.iteration += 1;
        }

        for _ in 0..10 {
            if self.core.solve_rtsp(true) {
                self.update_path();
                let explored_nodes = self.explored_nodes();
                self.core.log_data(explored_nodes);
            }
        }

        if self.core.shortest_path.is_empty() && self.core.shortest_path_cost.is_finite() {
            self.update_path();
        }

        self.core.make_result(self.explored_nodes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::{FakeMapLoader, MapLoader};

    fn test_config(system: u8) -> AlgorithmConfig {
        AlgorithmConfig::from_yaml_str(&format!(
            r#"
general: {{ system: {system}, pseudo: 0, log_data: 0, print_path: 0, max_iter: 5000, max_time: 30 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: 0, objective_ids: [1], target_id: 2 }}
map: {{ type: -1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 0, ga: {{ random_seed: 0, mutation_iter: 10, population: 10, generation: 1 }} }}
"#
        ))
        .unwrap()
    }

    #[test]
    fn bi_astar_finds_path_on_fake_map_1() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1],
            target: 2,
        };
        let mut planner = BiAstar::new(graph, dest, test_config(1)).unwrap();
        let result = planner.find_shortest_path().unwrap();
        assert_eq!(result.path.first().copied(), Some(0));
        assert_eq!(result.path.last().copied(), Some(2));
        assert!(result.cost > 0.0);
    }
}
