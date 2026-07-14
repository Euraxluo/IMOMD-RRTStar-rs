use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::sync::Arc;

use rand::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::config::AlgorithmConfig;
use crate::error::Result;
use crate::experiment::ExperimentRecord;
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::types::{Destinations, NodeId, PlanningResult};

use super::common::{pair_tree_idx, BaselineCore};
use super::BaselinePlanner;

#[derive(Eq, PartialEq)]
struct EpsilonState {
    epsilon: u64,
    node: NodeId,
}

impl PartialOrd for EpsilonState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for EpsilonState {
    fn cmp(&self, other: &Self) -> Ordering {
        f64::from_bits(self.epsilon)
            .partial_cmp(&f64::from_bits(other.epsilon))
            .unwrap_or(Ordering::Equal)
    }
}

struct AnaStarTree {
    id: usize,
    start_id: i32,
    goal_id: i32,
    start_root: NodeId,
    goal_root: NodeId,
    parent: FxHashMap<NodeId, NodeId>,
    cost: FxHashMap<NodeId, f64>,
    open_queue: BinaryHeap<EpsilonState>,
    g_bound: f64,
    epsilon: f64,
}

impl AnaStarTree {
    fn new(start_id: i32, goal_id: i32, start_root: NodeId, goal_root: NodeId) -> Self {
        let id = pair_tree_idx(start_id, goal_id);
        let mut parent = FxHashMap::default();
        parent.insert(start_root, start_root);
        let mut cost = FxHashMap::default();
        cost.insert(start_root, 0.0);
        let mut open_queue = BinaryHeap::new();
        open_queue.push(EpsilonState {
            epsilon: f64::INFINITY.to_bits(),
            node: start_root,
        });
        Self {
            id,
            start_id,
            goal_id,
            start_root,
            goal_root,
            parent,
            cost,
            open_queue,
            g_bound: f64::INFINITY,
            epsilon: f64::INFINITY,
        }
    }
}

/// ANA* baseline (maps to C++ `ANAStar`).
pub struct AnaStar {
    core: BaselineCore,
    tree_layers: Vec<AnaStarTree>,
    unexplored_tree: FxHashSet<usize>,
}

impl AnaStar {
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
                tree_layers.push(AnaStarTree::new(
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
            for i in 0..self.tree_layers.len() {
                self.unexplored_tree.insert(i);
            }
        }

        let tree_id = loop {
            let candidate = self.core.rng.gen_range(0..self.tree_layers.len());
            if self.unexplored_tree.contains(&candidate) {
                break candidate;
            }
        };

        self.expand_tree(tree_id);
        self.unexplored_tree.remove(&tree_id);
    }

    fn expand_tree(&mut self, tree_id: usize) {
        let graph = Arc::clone(&self.core.graph);
        let tree = &mut self.tree_layers[tree_id];

        while let Some(EpsilonState {
            epsilon: e_bits,
            node: state,
        }) = tree.open_queue.pop()
        {
            let e_value = f64::from_bits(e_bits);
            if e_value < tree.epsilon {
                let h = self.core.heuristic(state, tree.goal_root);
                tree.epsilon = (tree.g_bound - tree.cost[&state]) / (h + f64::MIN_POSITIVE);
            }

            if state == tree.goal_root {
                tree.g_bound = tree.cost[&state];
                break;
            }

            for (neighbor, weight) in graph.neighbors(state) {
                let new_cost = tree.cost[&state] + weight;
                let entry = tree.cost.entry(neighbor).or_insert(f64::INFINITY);
                if new_cost < *entry {
                    *entry = new_cost;
                    tree.parent.insert(neighbor, state);
                    let h = self.core.heuristic(neighbor, tree.goal_root);
                    let new_epsilon = (tree.g_bound - new_cost) / (h + f64::MIN_POSITIVE);
                    if new_epsilon > 1.0 {
                        tree.open_queue.push(EpsilonState {
                            epsilon: new_epsilon.to_bits(),
                            node: neighbor,
                        });
                    }
                }
            }
        }

        let mut updated_open = BinaryHeap::new();
        while let Some(EpsilonState { node: state, .. }) = tree.open_queue.pop() {
            let h = self.core.heuristic(state, tree.goal_root);
            let new_epsilon = (tree.g_bound - tree.cost[&state]) / (h + f64::MIN_POSITIVE);
            if new_epsilon > 1.0 {
                updated_open.push(EpsilonState {
                    epsilon: new_epsilon.to_bits(),
                    node: state,
                });
            }
        }
        tree.open_queue = updated_open;

        let si = tree.start_id as usize;
        let gi = tree.goal_id as usize;
        if tree.g_bound < self.core.distance_matrix[si][gi] - 0.1 {
            self.core.distance_matrix[si][gi] = tree.g_bound;
            self.core.distance_matrix[gi][si] = tree.g_bound;
            self.core.is_distance_matrix_updated = true;
        }

        self.core.connect_two_tree(tree.start_id, tree.goal_id);
        self.unexplored_tree.remove(&tree.id);
    }

    fn update_path(&mut self) {
        self.core.shortest_path.clear();
        for window in self.core.sequence_rtsp.windows(2) {
            let start_id = window[0] as i32;
            let goal_id = window[1] as i32;
            let tree_idx = pair_tree_idx(start_id, goal_id);
            let tree = &self.tree_layers[tree_idx];
            let mut node = tree.goal_root;

            if tree.start_id == start_id {
                let mut tmp_path = Vec::new();
                while node != tree.start_root {
                    node = tree.parent[&node];
                    tmp_path.push(node);
                }
                for &n in tmp_path.iter().rev() {
                    self.core.shortest_path.push(n);
                }
            } else {
                while node != tree.start_root {
                    self.core.shortest_path.push(node);
                    node = tree.parent[&node];
                }
            }
        }
        self.core
            .shortest_path
            .push(*self.core.destinations.last().unwrap());
    }

    fn solve_rtsp_and_update(&mut self, force: bool) {
        if self.core.solve_rtsp(force) {
            self.update_path();
            let explored_nodes = self.explored_nodes();
            self.core.log_data(explored_nodes);
        }
    }

    fn all_trees_exhausted(&self) -> bool {
        self.tree_layers.iter().all(|t| t.open_queue.is_empty())
    }

    fn explored_nodes(&self) -> usize {
        self.tree_layers.iter().map(|t| t.parent.len()).sum()
    }
}

impl BaselinePlanner for AnaStar {
    fn find_shortest_path(&mut self) -> Result<PlanningResult> {
        let mut expansion_done = false;

        while !self.core.timed_out() && !expansion_done {
            self.expand_tree_layers();
            self.solve_rtsp_and_update(false);
            self.core.iteration += 1;

            if self.core.iteration.is_multiple_of(1000) && self.all_trees_exhausted() {
                expansion_done = true;
            }
        }

        for _ in 0..10 {
            self.solve_rtsp_and_update(true);
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
    fn ana_star_finds_path_on_fake_map_1() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1],
            target: 2,
        };
        let mut planner = AnaStar::new(graph, dest, test_config(2)).unwrap();
        let result = planner.find_shortest_path().unwrap();
        assert_eq!(result.path.first().copied(), Some(0));
        assert_eq!(result.path.last().copied(), Some(2));
        assert!(result.cost > 0.0);
    }
}
