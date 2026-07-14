use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant};

use rand::prelude::*;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::config::AlgorithmConfig;
use crate::error::{PlannerError, Result};
use crate::experiment::{ExperimentLog, ExperimentRecord};
use crate::geo::{bearing, haversine_distance};
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::rrt::tree::RrtTree;
use crate::rrt::AnytimePlanner;
use crate::rtsp::EciGenSolver;
use crate::types::{
    Destinations, GraphUpdateStats, Location, NodeId, PlanningResult, StepResult, StepStatus,
};

fn connection_set_idx(a: i32, b: i32) -> usize {
    if a > b {
        (a * (a - 1) / 2 + b) as usize
    } else {
        (b * (b - 1) / 2 + a) as usize
    }
}

/// Anytime IMOMD-RRT* planner (maps to C++ `ImomdRRT`).
pub struct ImomdRrtStar {
    graph: Arc<AdjacencyGraph>,
    config: AlgorithmConfig,
    destinations: Destinations,
    destination_nodes: Vec<NodeId>,
    source_tree_id: i32,
    target_tree_id: i32,

    iteration: usize,
    rng: StdRng,
    start_time: Instant,

    tree_layers: Vec<RrtTree>,
    probability_matrix: Vec<Vec<f64>>,
    expandables_min_heuristic_matrix: Vec<Vec<(NodeId, f64)>>,
    distance_matrix: Vec<Vec<f64>>,
    connection_node_matrix: Vec<Vec<NodeId>>,
    connection_nodes_set: Vec<FxHashSet<NodeId>>,

    disjoint_set_parent: Vec<i32>,
    disjoint_set_children: FxHashMap<i32, Vec<i32>>,
    is_connected_graph: bool,

    eci_gen_solver: EciGenSolver,
    sequence_of_tree_id_rtsp: Vec<usize>,

    shortest_path_cost: f64,
    shortest_path: Vec<NodeId>,
    is_distance_matrix_updated: bool,
    is_merge_done: bool,

    best: Option<PlanningResult>,
    finished: bool,
    experiment_log: Option<ExperimentLog>,
}

impl ImomdRrtStar {
    /// Shared ownership handle for the active road graph.
    pub fn graph_arc(&self) -> Arc<AdjacencyGraph> {
        Arc::clone(&self.graph)
    }

    pub fn new(
        graph: Arc<AdjacencyGraph>,
        destinations: Destinations,
        config: AlgorithmConfig,
    ) -> Result<Self> {
        config.validate()?;
        let destination_nodes = destinations.all_nodes();
        let mut unique_destinations = FxHashSet::default();
        for &node in &destination_nodes {
            if graph.location(node).is_none() {
                return Err(PlannerError::NodeNotFound(node));
            }
            if !unique_destinations.insert(node) {
                return Err(PlannerError::Config(format!(
                    "duplicate destination node: {node}"
                )));
            }
        }

        let dest_count = destination_nodes.len();
        let source_tree_id = 0;
        let target_tree_id = (dest_count - 1) as i32;

        let mut tree_layers: Vec<RrtTree> = Vec::with_capacity(dest_count);
        let mut probability_matrix = vec![vec![0.0; dest_count]; dest_count];
        let mut expandables_min_heuristic_matrix =
            vec![vec![(usize::MAX, f64::INFINITY); dest_count]; dest_count];
        let mut distance_matrix = vec![vec![0.0; dest_count]; dest_count];
        let mut connection_node_matrix = vec![vec![usize::MAX; dest_count]; dest_count];
        let mut connection_nodes_set = Vec::new();

        let mut bearing_matrix: Vec<Vec<(f64, usize)>> = vec![Vec::new(); dest_count];
        let mut inversed_haversine_matrix = vec![vec![0.0; dest_count]; dest_count];
        let mut sum_haversine = vec![0.0; dest_count];

        let disjoint_set_parent: Vec<i32> = (0..dest_count as i32).collect();
        let mut disjoint_set_children: FxHashMap<i32, Vec<i32>> = FxHashMap::default();

        for i in 0..dest_count {
            let root = destination_nodes[i];
            let mut tree = RrtTree::new(i as i32, root);
            let others: Vec<(i32, NodeId)> = tree_layers.iter().map(|t| (t.id, t.root)).collect();
            Self::update_expandables(
                &mut tree,
                &others,
                &mut expandables_min_heuristic_matrix,
                &graph,
                root,
                true,
            );
            tree_layers.push(tree);
            disjoint_set_children.insert(i as i32, Vec::new());

            for j in (i + 1)..dest_count {
                let loc_i = graph.location(destination_nodes[i]).unwrap();
                let loc_j = graph.location(destination_nodes[j]).unwrap();

                bearing_matrix[i].push((
                    bearing(
                        loc_i.latitude,
                        loc_i.longitude,
                        loc_j.latitude,
                        loc_j.longitude,
                    ),
                    j,
                ));
                bearing_matrix[j].push((
                    bearing(
                        loc_j.latitude,
                        loc_j.longitude,
                        loc_i.latitude,
                        loc_i.longitude,
                    ),
                    i,
                ));

                let haversine = haversine_distance(
                    loc_i.latitude,
                    loc_i.longitude,
                    loc_j.latitude,
                    loc_j.longitude,
                );
                if haversine <= f64::EPSILON {
                    return Err(PlannerError::Config(format!(
                        "destination nodes {} and {} have identical coordinates",
                        destination_nodes[i], destination_nodes[j]
                    )));
                }
                let inv_h = 1.0 / haversine;
                inversed_haversine_matrix[i][j] = inv_h;
                inversed_haversine_matrix[j][i] = inv_h;
                sum_haversine[i] += inv_h;
                sum_haversine[j] += inv_h;

                distance_matrix[i][j] = f64::INFINITY;
                distance_matrix[j][i] = f64::INFINITY;
                connection_node_matrix[i][j] = usize::MAX;
                connection_node_matrix[j][i] = usize::MAX;
                connection_nodes_set.push(FxHashSet::default());
            }
        }

        if dest_count > 3 {
            for i in 0..dest_count {
                bearing_matrix[i].sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
                let mut sum_bearing = 0.0;
                for j in 0..dest_count - 1 {
                    let curr_angle = bearing_matrix[i][j].0;
                    let prev_angle = bearing_matrix[i][(j + dest_count - 2) % (dest_count - 1)].0;
                    let next_angle = bearing_matrix[i][(j + 1) % (dest_count - 1)].0;

                    let mut angle_diff_prev = curr_angle - prev_angle;
                    if angle_diff_prev < 0.0 {
                        angle_diff_prev += 2.0 * std::f64::consts::PI;
                    }
                    let mut angle_diff_next = next_angle - curr_angle;
                    if angle_diff_next < 0.0 {
                        angle_diff_next += 2.0 * std::f64::consts::PI;
                    }

                    let idx = bearing_matrix[i][j].1;
                    probability_matrix[i][idx] = angle_diff_prev + angle_diff_next;
                    sum_bearing += probability_matrix[i][idx];
                }
                for probability in &mut probability_matrix[i] {
                    *probability /= sum_bearing;
                }
            }
        }

        for i in 0..dest_count {
            for j in 0..dest_count {
                probability_matrix[i][j] += inversed_haversine_matrix[i][j] / sum_haversine[i];
            }
        }

        let mut sum_probability = vec![0.0; dest_count];
        for i in 0..dest_count {
            for j in (i + 1)..dest_count {
                probability_matrix[i][j] += probability_matrix[j][i];
                probability_matrix[j][i] = probability_matrix[i][j];
                sum_probability[i] += probability_matrix[i][j];
                sum_probability[j] += probability_matrix[j][i];
            }
            for probability in &mut probability_matrix[i] {
                *probability /= sum_probability[i];
            }
        }

        let seed = if config.rrt_params.random_seed != 0 {
            rand::thread_rng().gen()
        } else {
            0
        };

        let eci_gen_solver = EciGenSolver::new(&config.rtsp_settings);
        let experiment_log = ExperimentLog::from_enabled(
            config.general.log_data != 0,
            Some(std::path::PathBuf::from("experiments/imomd_latest.csv")),
        )?;

        Ok(Self {
            graph,
            config,
            destinations,
            destination_nodes,
            source_tree_id,
            target_tree_id,
            iteration: 0,
            rng: StdRng::seed_from_u64(seed),
            start_time: Instant::now(),
            tree_layers,
            probability_matrix,
            expandables_min_heuristic_matrix,
            distance_matrix,
            connection_node_matrix,
            connection_nodes_set,
            disjoint_set_parent,
            disjoint_set_children,
            is_connected_graph: false,
            eci_gen_solver,
            sequence_of_tree_id_rtsp: Vec::new(),
            shortest_path_cost: f64::INFINITY,
            shortest_path: Vec::new(),
            is_distance_matrix_updated: false,
            is_merge_done: false,
            best: None,
            finished: false,
            experiment_log,
        })
    }

    pub fn experiment_records(&self) -> &[ExperimentRecord] {
        self.experiment_log
            .as_ref()
            .map(|log| log.records())
            .unwrap_or(&[])
    }

    pub fn destinations(&self) -> &Destinations {
        &self.destinations
    }

    pub fn best_solution(&self) -> Option<&PlanningResult> {
        self.best.as_ref()
    }

    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// Clear the terminal flag so another anytime budget slice can run.
    /// When exploration has already exhausted destination trees, reopen them
    /// with a fresh uniform sampling bias so RRT* can keep expanding/rewiring.
    pub fn resume_search(&mut self) {
        let reopen = self.exploration_exhausted();
        self.finished = false;
        // Measure wall-clock `max_time` from this resume so pause/resume slices
        // are not killed by the original planner creation timestamp.
        self.start_time = Instant::now();
        if reopen {
            self.reopen_exploration();
        }
    }

    fn reopen_exploration(&mut self) {
        let n = self.tree_layers.len();
        if n <= 1 {
            return;
        }
        let uniform = 1.0 / (n - 1) as f64;
        for i in 0..n {
            self.tree_layers[i].is_done = false;
            for j in 0..n {
                self.probability_matrix[i][j] = if i == j { 0.0 } else { uniform };
            }
        }
    }

    /// Run additional RTSP polishing passes on the current best path.
    pub fn refine_solution(&mut self, rounds: usize) {
        for _ in 0..rounds {
            self.solve_rtsp(true);
        }
    }

    /// True when every destination tree has stopped expanding and a path exists.
    pub fn exploration_exhausted(&self) -> bool {
        self.best.is_some() && self.tree_layers.iter().all(|tree| tree.is_done)
    }

    pub fn step(&mut self) -> Result<StepResult> {
        if self.finished {
            return Ok(StepResult {
                status: StepStatus::Finished,
                iteration: self.iteration,
                best_cost: self.best.as_ref().map(|b| b.cost),
            });
        }

        let was_connected = self.is_connected_graph;
        let previous_best_cost = self.best.as_ref().map(|best| best.cost);
        self.expand_tree_layers();

        // Publish a feasible path as soon as trees connect, then keep refining
        // whenever the inter-tree distance matrix improves.
        if !was_connected && self.is_connected_graph {
            self.solve_rtsp(true);
        } else if self.is_distance_matrix_updated && self.iteration.is_multiple_of(20) {
            self.solve_rtsp(false);
        } else if self.iteration.is_multiple_of(100) {
            self.solve_rtsp(false);
        }

        self.iteration += 1;
        let expansion_finished = self.iteration.is_multiple_of(100)
            && self.is_connected_graph
            && self.best.is_some()
            && self.tree_layers.iter().all(|tree| tree.is_done);
        // Anytime: do not terminate solely because sampling bias says trees are
        // "done". Keep expanding/rewiring until max_iter / max_time.
        if expansion_finished {
            for _ in 0..3 {
                self.solve_rtsp(true);
            }
        }
        if self.iteration > self.config.general.max_iter
            || self.start_time.elapsed().as_secs() >= self.config.general.max_time
        {
            for _ in 0..10 {
                self.solve_rtsp(true);
            }
            self.finished = true;
            if let Some(best) = self.best.clone() {
                return Ok(StepResult {
                    status: StepStatus::Finished,
                    iteration: self.iteration,
                    best_cost: Some(best.cost),
                });
            }
        }

        let current_best_cost = self.best.as_ref().map(|best| best.cost);
        let path_improved = match (previous_best_cost, current_best_cost) {
            (None, Some(_)) => true,
            (Some(previous), Some(current)) => current < previous,
            _ => false,
        };
        Ok(StepResult {
            status: if path_improved {
                StepStatus::PathImproved
            } else if !was_connected && self.is_connected_graph {
                StepStatus::Connected
            } else {
                StepStatus::Expanded
            },
            iteration: self.iteration,
            best_cost: self.best.as_ref().map(|b| b.cost),
        })
    }

    pub fn run_until(&mut self, deadline: Instant) -> Result<PlanningResult> {
        while Instant::now() < deadline && !self.finished {
            self.step()?;
        }
        // A caller deadline is a pause point, not an algorithm terminal state.
        // Force one final RTSP evaluation for this time slice while preserving
        // the trees so a later run_for/run_until call can continue improving.
        if !self.finished {
            for _ in 0..10 {
                self.solve_rtsp(true);
            }
        }
        self.best.clone().ok_or(PlannerError::Disconnected(
            self.destinations.source,
            self.destinations.target,
        ))
    }

    pub fn run_for(&mut self, duration: Duration) -> Result<PlanningResult> {
        self.run_until(Instant::now() + duration)
    }

    /// Replace edge weights/topology while retaining every still-valid branch
    /// of the destination-rooted RRT* trees. Costs, expandables, inter-tree
    /// connections and the RTSP solution are rebuilt against the new graph.
    pub fn update_graph(&mut self, graph: Arc<AdjacencyGraph>) -> Result<GraphUpdateStats> {
        if graph.node_count() != self.graph.node_count() {
            return Err(PlannerError::Config(format!(
                "dynamic graph node count changed from {} to {}",
                self.graph.node_count(),
                graph.node_count()
            )));
        }
        for node in 0..graph.node_count() {
            let old = self
                .graph
                .location(node)
                .ok_or(PlannerError::NodeNotFound(node))?;
            let new = graph
                .location(node)
                .ok_or(PlannerError::NodeNotFound(node))?;
            if old.id != new.id
                || old.latitude.to_bits() != new.latitude.to_bits()
                || old.longitude.to_bits() != new.longitude.to_bits()
            {
                return Err(PlannerError::Config(format!(
                    "dynamic graph node {node} changed identity or coordinates"
                )));
            }
        }

        let previous_tree_nodes = self.tree_size();
        let retained_trees: Vec<RrtTree> = self
            .tree_layers
            .iter()
            .map(|tree| Self::retain_tree_on_graph(tree, &graph))
            .collect();
        let retained_tree_nodes = retained_trees.iter().map(|tree| tree.parent.len()).sum();

        // Reuse normal construction to obtain fresh probability/config/RTSP
        // state, then transplant the retained search trees and RNG stream.
        let mut refreshed = Self::new(
            Arc::clone(&graph),
            self.destinations.clone(),
            self.config.clone(),
        )?;
        refreshed.rng = self.rng.clone();
        refreshed.tree_layers = retained_trees;
        refreshed.experiment_log = self.experiment_log.take();
        refreshed.rebuild_expandable_heuristics();
        refreshed.rebuild_connections();
        refreshed.solve_rtsp(true);

        *self = refreshed;
        Ok(GraphUpdateStats {
            previous_tree_nodes,
            retained_tree_nodes,
            pruned_tree_nodes: previous_tree_nodes.saturating_sub(retained_tree_nodes),
        })
    }

    fn retain_tree_on_graph(tree: &RrtTree, graph: &AdjacencyGraph) -> RrtTree {
        let mut retained = RrtTree::new(tree.id, tree.root);
        let mut queue = VecDeque::from([tree.root]);
        while let Some(parent) = queue.pop_front() {
            let Some(children) = tree.children.get(&parent) else {
                continue;
            };
            for &child in children {
                if retained.is_visited(child) {
                    continue;
                }
                if let Some(weight) = graph.edge_weight(parent, child) {
                    retained.add_child(parent, child, weight);
                    retained.children.entry(child).or_default();
                    queue.push_back(child);
                }
            }
        }

        let visited: FxHashSet<NodeId> = retained.parent.keys().copied().collect();
        for &node in &visited {
            for (neighbor, _) in graph.neighbors(node) {
                if !visited.contains(&neighbor) {
                    retained.expandables.insert(neighbor);
                }
            }
        }
        retained.is_done = false;
        retained
    }

    fn rebuild_expandable_heuristics(&mut self) {
        let count = self.tree_layers.len();
        self.expandables_min_heuristic_matrix =
            vec![vec![(usize::MAX, f64::INFINITY); count]; count];
        for tree_idx in 0..count {
            for other_idx in 0..count {
                if tree_idx == other_idx {
                    continue;
                }
                let root = self.tree_layers[tree_idx].root;
                let other_root = self.tree_layers[other_idx].root;
                for &expandable in &self.tree_layers[tree_idx].expandables {
                    let heuristic =
                        Self::heuristic_distance(&self.graph, root, other_root, expandable);
                    if heuristic < self.expandables_min_heuristic_matrix[tree_idx][other_idx].1 {
                        self.expandables_min_heuristic_matrix[tree_idx][other_idx] =
                            (expandable, heuristic);
                    }
                }
            }
        }
    }

    fn rebuild_connections(&mut self) {
        let count = self.tree_layers.len();
        self.distance_matrix = vec![vec![0.0; count]; count];
        self.connection_node_matrix = vec![vec![usize::MAX; count]; count];
        self.connection_nodes_set = (0..count * (count - 1) / 2)
            .map(|_| FxHashSet::default())
            .collect();
        for i in 0..count {
            for j in (i + 1)..count {
                self.distance_matrix[i][j] = f64::INFINITY;
                self.distance_matrix[j][i] = f64::INFINITY;
            }
        }
        self.disjoint_set_parent = (0..count as i32).collect();
        self.disjoint_set_children = (0..count as i32).map(|id| (id, Vec::new())).collect();
        self.is_connected_graph = false;
        self.is_distance_matrix_updated = false;
        self.is_merge_done = false;
        self.sequence_of_tree_id_rtsp.clear();
        self.shortest_path.clear();
        self.shortest_path_cost = f64::INFINITY;
        self.best = None;
        self.finished = false;

        let visited_by_tree: Vec<Vec<NodeId>> = self
            .tree_layers
            .iter()
            .map(|tree| tree.parent.keys().copied().collect())
            .collect();
        for (tree_idx, nodes) in visited_by_tree.iter().enumerate() {
            for &node in nodes {
                self.update_connection_tree(tree_idx, node);
            }
        }
    }

    pub fn init_trees(&self) -> Vec<RrtTree> {
        self.tree_layers.clone()
    }

    fn other_tree_roots(&self) -> Vec<(i32, NodeId)> {
        self.tree_layers.iter().map(|t| (t.id, t.root)).collect()
    }

    fn expand_tree_layers(&mut self) {
        let count = self.tree_layers.len();
        for i in 0..count {
            // Keep expanding even after `is_done` so anytime search can rewire
            // toward better paths; `is_done` only reflects sampling-bias state.
            if self.tree_layers[i].expandables.is_empty() {
                continue;
            }
            let random_point = self.select_random_vertex(i);
            let x_new = self.steer_new_node(i, &random_point);
            self.connect_new_node(i, x_new);
            let others = self.other_tree_roots();
            Self::update_expandables(
                &mut self.tree_layers[i],
                &others,
                &mut self.expandables_min_heuristic_matrix,
                &self.graph,
                x_new,
                true,
            );
            self.rewire_tree(i, x_new);
            self.update_selection_probability(i, &random_point);
            self.update_connection_tree(i, x_new);
        }
    }

    fn select_random_vertex(&mut self, tree_idx: usize) -> Location {
        let tree = &self.tree_layers[tree_idx];
        if self.rng.gen::<f64>() < self.config.rrt_params.goal_bias {
            let probs = &self.probability_matrix[tree_idx];
            let mut rand_val = self.rng.gen::<f64>();
            let mut i = 0;
            while i < probs.len() && rand_val >= 0.0 {
                rand_val -= probs[i];
                i += 1;
            }
            let dest_idx = i.saturating_sub(1).min(self.tree_layers.len().saturating_sub(1));
            let random_dest = self.tree_layers[dest_idx].root;
            let mut random_point = self.graph.location(random_dest).unwrap().clone();

            if self.tree_layers[tree_idx].is_visited(random_dest) {
                let ratio: f64 = self.rng.gen_range(0.0..0.5);
                let root_loc = self.graph.location(tree.root).unwrap();
                random_point.id = random_dest;
                random_point.latitude =
                    ratio * random_point.latitude + (1.0 - ratio) * root_loc.latitude;
                random_point.longitude =
                    ratio * random_point.longitude + (1.0 - ratio) * root_loc.longitude;
            }
            random_point
        } else {
            let node_id = self.rng.gen_range(0..self.graph.node_count());
            self.graph.location(node_id).unwrap().clone()
        }
    }

    fn steer_new_node(&mut self, tree_idx: usize, random_point: &Location) -> NodeId {
        let tree = &self.tree_layers[tree_idx];
        // The nearest-expandable scan dominates large road-network steps and
        // is read-only, so it can be parallelized without locking the shared
        // distance/connection matrices. Tie-breaking by node id keeps results
        // deterministic across Rayon scheduling orders.
        let mut x_new = tree
            .expandables
            .par_iter()
            .map(|&expandable| {
                let loc = self.graph.location(expandable).unwrap();
                let distance = haversine_distance(
                    loc.latitude,
                    loc.longitude,
                    random_point.latitude,
                    random_point.longitude,
                );
                (expandable, distance)
            })
            .min_by(|(node_a, distance_a), (node_b, distance_b)| {
                distance_a
                    .total_cmp(distance_b)
                    .then_with(|| node_a.cmp(node_b))
            })
            .expect("steer_new_node requires a non-empty expandable set")
            .0;

        if self.graph.neighbors(x_new).len() == 2 {
            let others = self.other_tree_roots();
            Self::update_expandables(
                &mut self.tree_layers[tree_idx],
                &others,
                &mut self.expandables_min_heuristic_matrix,
                &self.graph,
                x_new,
                false,
            );
        }

        let mut is_jps_done = false;
        while self.graph.neighbors(x_new).len() == 2 && !is_jps_done {
            let neighbors: Vec<_> = self.graph.neighbors(x_new);
            let x_1 = neighbors[0].0;
            let x_2 = neighbors[1].0;

            if self.tree_layers[tree_idx].is_visited(x_1) {
                if self.tree_layers[tree_idx].is_visited(x_2) {
                    is_jps_done = true;
                } else {
                    let cost = self.tree_layers[tree_idx].cost[&x_1]
                        + self.graph.edge_weight(x_1, x_new).unwrap();
                    let tree = &mut self.tree_layers[tree_idx];
                    tree.parent.insert(x_new, x_1);
                    tree.children.entry(x_1).or_default().insert(x_new);
                    tree.children.entry(x_new).or_default();
                    tree.cost.insert(x_new, cost);
                    self.update_connection_tree(tree_idx, x_new);
                    x_new = x_2;
                }
            } else {
                let cost = self.tree_layers[tree_idx].cost[&x_2]
                    + self.graph.edge_weight(x_2, x_new).unwrap();
                let tree = &mut self.tree_layers[tree_idx];
                tree.parent.insert(x_new, x_2);
                tree.children.entry(x_2).or_default().insert(x_new);
                tree.children.entry(x_new).or_default();
                tree.cost.insert(x_new, cost);
                self.update_connection_tree(tree_idx, x_new);
                x_new = x_1;
            }
        }

        x_new
    }

    fn connect_new_node(&mut self, tree_idx: usize, x_new: NodeId) {
        let tree = &mut self.tree_layers[tree_idx];
        let mut x_parent = tree.root;
        let mut min_cost = f64::INFINITY;
        let mut is_connectable = false;

        for (x_near, weight) in self.graph.neighbors(x_new) {
            if tree.is_visited(x_near) {
                let cost_x_new = tree.cost[&x_near] + weight;
                if cost_x_new < min_cost {
                    x_parent = x_near;
                    min_cost = cost_x_new;
                    is_connectable = true;
                }
            }
        }

        if is_connectable {
            tree.parent.insert(x_new, x_parent);
            tree.children.entry(x_parent).or_default().insert(x_new);
            tree.cost.insert(x_new, min_cost);
        }
    }

    fn update_expandables(
        tree: &mut RrtTree,
        other_trees: &[(i32, NodeId)],
        heuristic_matrix: &mut [Vec<(NodeId, f64)>],
        graph: &AdjacencyGraph,
        x_new: NodeId,
        search_neighbor: bool,
    ) {
        let tree_id = tree.id as usize;
        tree.expandables.remove(&x_new);

        for &(other_id_i32, other_root) in other_trees {
            if other_id_i32 == tree.id {
                continue;
            }
            let other_id = other_id_i32 as usize;
            if x_new == heuristic_matrix[tree_id][other_id].0 {
                heuristic_matrix[tree_id][other_id] = (usize::MAX, f64::INFINITY);
                for &expandable in &tree.expandables {
                    let h = Self::heuristic_distance(graph, tree.root, other_root, expandable);
                    if h < heuristic_matrix[tree_id][other_id].1 {
                        heuristic_matrix[tree_id][other_id] = (expandable, h);
                    }
                }
            }
        }

        if !search_neighbor {
            return;
        }

        for (x_near, _) in graph.neighbors(x_new) {
            if !tree.is_visited(x_near) {
                tree.expandables.insert(x_near);
                for &(other_id_i32, other_root) in other_trees {
                    if other_id_i32 == tree.id {
                        continue;
                    }
                    let other_id = other_id_i32 as usize;
                    let h = Self::heuristic_distance(graph, tree.root, other_root, x_near);
                    if h < heuristic_matrix[tree_id][other_id].1 {
                        heuristic_matrix[tree_id][other_id] = (x_near, h);
                    }
                }
            }
        }
    }

    fn heuristic_distance(
        graph: &AdjacencyGraph,
        root_a: NodeId,
        root_b: NodeId,
        node: NodeId,
    ) -> f64 {
        let loc = graph.location(node).unwrap();
        let loc_a = graph.location(root_a).unwrap();
        let loc_b = graph.location(root_b).unwrap();
        haversine_distance(loc.latitude, loc.longitude, loc_a.latitude, loc_a.longitude)
            + haversine_distance(loc.latitude, loc.longitude, loc_b.latitude, loc_b.longitude)
    }

    fn rewire_tree(&mut self, tree_idx: usize, x_new: NodeId) {
        let neighbors: Vec<_> = self.graph.neighbors(x_new);
        for (x_near, weight) in neighbors {
            let tree = &self.tree_layers[tree_idx];
            if tree.is_visited(x_near) && tree.parent.get(&x_new) != Some(&x_near) {
                let new_cost = tree.cost[&x_new] + weight;
                if tree.cost[&x_near] > new_cost {
                    let updated_nodes = {
                        let tree = &mut self.tree_layers[tree_idx];
                        let parent = tree.parent[&x_near];
                        tree.children.get_mut(&parent).unwrap().remove(&x_near);
                        tree.parent.insert(x_near, x_new);
                        tree.children.entry(x_new).or_default().insert(x_near);
                        tree.update_cost(x_near, new_cost)
                    };

                    for node in updated_nodes.iter().rev() {
                        if *node != x_near {
                            self.rewire_tree(tree_idx, *node);
                        }
                    }
                    self.update_connection_tree(tree_idx, x_near);
                }
            }
        }
    }

    fn update_selection_probability(&mut self, tree_idx: usize, random_point: &Location) {
        let dest_pos = self
            .destination_nodes
            .iter()
            .position(|&d| d == random_point.id);
        let Some(other_tree_id) = dest_pos else {
            return;
        };

        if self.expandables_min_heuristic_matrix[tree_idx][other_tree_id].1
            <= self.distance_matrix[tree_idx][other_tree_id]
        {
            return;
        }

        self.probability_matrix[tree_idx][other_tree_id] = 0.0;
        self.probability_matrix[other_tree_id][tree_idx] = 0.0;

        self.tree_layers[tree_idx].is_done =
            self.probability_matrix[tree_idx].iter().all(|&p| p == 0.0);
        self.tree_layers[other_tree_id].is_done = self.probability_matrix[other_tree_id]
            .iter()
            .all(|&p| p == 0.0);

        let sum_tree: f64 = self.probability_matrix[tree_idx].iter().sum();
        let sum_other: f64 = self.probability_matrix[other_tree_id].iter().sum();
        if sum_tree > 0.0 && sum_tree.is_finite() {
            for probability in &mut self.probability_matrix[tree_idx] {
                *probability /= sum_tree;
            }
        }
        if sum_other > 0.0 && sum_other.is_finite() {
            for probability in &mut self.probability_matrix[other_tree_id] {
                *probability /= sum_other;
            }
        }
    }

    fn update_connection_tree(&mut self, tree_idx: usize, x_new: NodeId) {
        let tree_id = self.tree_layers[tree_idx].id;
        let tree_cost = self.tree_layers[tree_idx].cost.clone();
        let tree_parent = self.tree_layers[tree_idx].parent.clone();
        let pseudo_mode = self.config.general.pseudo != 0;

        for other_idx in 0..self.tree_layers.len() {
            if self.tree_layers[other_idx].id == tree_id {
                continue;
            }
            if !self.tree_layers[other_idx].is_visited(x_new) {
                continue;
            }

            let other_id = self.tree_layers[other_idx].id;
            let set_idx = connection_set_idx(tree_id, other_id);

            if self.connection_nodes_set[set_idx].is_empty() {
                self.connection_nodes_set[set_idx].insert(x_new);
                self.connect_two_tree(tree_id, other_id);
            }

            let distance = tree_cost[&x_new] + self.tree_layers[other_idx].cost[&x_new];
            let ti = tree_id as usize;
            let oi = other_id as usize;
            if distance < self.distance_matrix[ti][oi] - 0.1 {
                self.distance_matrix[ti][oi] = distance;
                self.distance_matrix[oi][ti] = distance;
                self.connection_node_matrix[ti][oi] = x_new;
                self.connection_node_matrix[oi][ti] = x_new;
                self.is_distance_matrix_updated = true;
            }

            if pseudo_mode {
                let x_parent = tree_parent[&x_new];
                let other = &self.tree_layers[other_idx];
                if !other.is_visited(x_parent)
                    || (other.parent.get(&x_parent) != Some(&x_new)
                        && other.parent.get(&x_new) != Some(&x_parent))
                {
                    self.connection_nodes_set[set_idx].insert(x_new);
                }
            }
        }
    }

    fn connect_two_tree(&mut self, tree1_id: i32, tree2_id: i32) {
        if self.is_connected_graph {
            return;
        }

        let mut min_tree_id = self.disjoint_set_parent[tree1_id as usize];
        let mut max_tree_id = self.disjoint_set_parent[tree2_id as usize];
        if max_tree_id == min_tree_id {
            return;
        }
        if max_tree_id < min_tree_id {
            std::mem::swap(&mut min_tree_id, &mut max_tree_id);
        }

        self.disjoint_set_parent[max_tree_id as usize] = min_tree_id;
        self.disjoint_set_children
            .entry(min_tree_id)
            .or_default()
            .push(max_tree_id);

        let children: Vec<i32> = self
            .disjoint_set_children
            .get(&max_tree_id)
            .cloned()
            .unwrap_or_default();
        for child in children {
            self.disjoint_set_parent[child as usize] = min_tree_id;
            self.disjoint_set_children
                .entry(min_tree_id)
                .or_default()
                .push(child);
        }
        self.disjoint_set_children.remove(&max_tree_id);

        self.is_connected_graph = self.disjoint_set_parent[1..]
            .iter()
            .all(|&p| p == self.disjoint_set_parent[0]);
    }

    fn merge_pseudo_trees(&mut self) {
        let mut unmerged: FxHashSet<i32> = FxHashSet::from_iter(1..self.target_tree_id);

        while !unmerged.is_empty() {
            let mut pseudo_tree_merge = 0;
            let mut max_connection_number = 0;
            for &pseudo_tree_id in &unmerged {
                let set_idx = connection_set_idx(pseudo_tree_id, self.source_tree_id);
                let connection_number = self.connection_nodes_set[set_idx].len();
                if connection_number > max_connection_number {
                    max_connection_number = connection_number;
                    pseudo_tree_merge = pseudo_tree_id;
                }
            }
            if max_connection_number > 0 {
                self.merge_two_tree(self.source_tree_id, pseudo_tree_merge);
                unmerged.remove(&pseudo_tree_merge);
            }

            let mut max_connection_number = 0;
            let mut pseudo_tree_merge = 0;
            for &pseudo_tree_id in &unmerged {
                let set_idx = connection_set_idx(self.target_tree_id, pseudo_tree_id);
                let connection_number = self.connection_nodes_set[set_idx].len();
                if connection_number > max_connection_number {
                    max_connection_number = connection_number;
                    pseudo_tree_merge = pseudo_tree_id;
                }
            }
            if max_connection_number > 0 {
                self.merge_two_tree(self.target_tree_id, pseudo_tree_merge);
                unmerged.remove(&pseudo_tree_merge);
            }
        }

        let set_idx = connection_set_idx(self.target_tree_id, 0);
        let mut min_distance =
            self.distance_matrix[self.source_tree_id as usize][self.target_tree_id as usize];
        for &connection_node in &self.connection_nodes_set[set_idx] {
            let distance = self.tree_layers[self.source_tree_id as usize].cost[&connection_node]
                + self.tree_layers[self.target_tree_id as usize].cost[&connection_node];
            if distance < min_distance {
                min_distance = distance;
                self.distance_matrix[self.source_tree_id as usize][self.target_tree_id as usize] =
                    distance;
                self.distance_matrix[self.target_tree_id as usize][self.source_tree_id as usize] =
                    distance;
                self.connection_node_matrix[self.source_tree_id as usize]
                    [self.target_tree_id as usize] = connection_node;
                self.connection_node_matrix[self.target_tree_id as usize]
                    [self.source_tree_id as usize] = connection_node;
                self.is_distance_matrix_updated = true;
            }
        }
        self.is_merge_done = true;
    }

    fn merge_two_tree(&mut self, parent_tree_id: i32, child_tree_id: i32) {
        let set_idx = connection_set_idx(parent_tree_id, child_tree_id);
        let connection_nodes: Vec<NodeId> =
            self.connection_nodes_set[set_idx].iter().copied().collect();

        let mut merged_nodes: FxHashSet<NodeId> = FxHashSet::default();
        for connection_node in connection_nodes {
            let mut current_node = connection_node;
            let child_root = self.tree_layers[child_tree_id as usize].root;
            let mut is_optimal = true;

            while current_node != child_root && is_optimal {
                let x_new = self.tree_layers[child_tree_id as usize].parent[&current_node];
                if self.tree_layers[parent_tree_id as usize].is_visited(x_new) {
                    if merged_nodes.contains(&x_new) {
                        is_optimal = false;
                    }
                } else {
                    self.connect_new_node(parent_tree_id as usize, x_new);
                    let others = self.other_tree_roots();
                    Self::update_expandables(
                        &mut self.tree_layers[parent_tree_id as usize],
                        &others,
                        &mut self.expandables_min_heuristic_matrix,
                        &self.graph,
                        x_new,
                        true,
                    );
                    self.rewire_tree(parent_tree_id as usize, x_new);
                    self.update_connection_tree(parent_tree_id as usize, x_new);
                    merged_nodes.insert(x_new);
                }
                current_node = x_new;
            }
        }

        let child_parent: Vec<(NodeId, NodeId)> = self.tree_layers[child_tree_id as usize]
            .parent
            .iter()
            .map(|(&k, &v)| (k, v))
            .collect();

        for (node, _) in child_parent {
            if self.tree_layers[parent_tree_id as usize].is_visited(node) {
                continue;
            }
            let mut current_node = node;
            let mut branch = vec![current_node];
            while !self.tree_layers[parent_tree_id as usize]
                .expandables
                .contains(&current_node)
            {
                current_node = self.tree_layers[child_tree_id as usize].parent[&current_node];
                branch.push(current_node);
            }
            for &x_new in branch.iter().rev() {
                if !self.tree_layers[parent_tree_id as usize].is_visited(x_new) {
                    self.connect_new_node(parent_tree_id as usize, x_new);
                    let others = self.other_tree_roots();
                    Self::update_expandables(
                        &mut self.tree_layers[parent_tree_id as usize],
                        &others,
                        &mut self.expandables_min_heuristic_matrix,
                        &self.graph,
                        x_new,
                        true,
                    );
                    self.rewire_tree(parent_tree_id as usize, x_new);
                    self.update_connection_tree(parent_tree_id as usize, x_new);
                }
            }
        }

        for other in 0..self.tree_layers.len() {
            let other_id = self.tree_layers[other].id;
            if other_id == child_tree_id || other_id == parent_tree_id {
                continue;
            }
            let child_set_idx = connection_set_idx(child_tree_id, other_id);
            let parent_set_idx = connection_set_idx(parent_tree_id, other_id);
            let nodes: Vec<NodeId> = self.connection_nodes_set[child_set_idx]
                .iter()
                .copied()
                .collect();
            for connection_node in nodes {
                if !self.tree_layers[parent_tree_id as usize].is_visited(connection_node) {
                    self.connection_nodes_set[parent_set_idx].insert(connection_node);
                }
            }
        }

        self.tree_layers[child_tree_id as usize].is_done = true;
    }

    fn update_path(&mut self) {
        self.shortest_path.clear();
        for window in self.sequence_of_tree_id_rtsp.windows(2) {
            let current_tree_id = window[0];
            let parent_tree_id = window[1];
            let connection_node = self.connection_node_matrix[current_tree_id][parent_tree_id];
            if connection_node == usize::MAX {
                return;
            }

            let mut tmp_path = Vec::new();
            let mut node = connection_node;
            while node != self.tree_layers[current_tree_id].root {
                node = self.tree_layers[current_tree_id].parent[&node];
                tmp_path.push(node);
            }
            for &n in tmp_path.iter().rev() {
                self.shortest_path.push(n);
            }

            node = connection_node;
            while node != self.tree_layers[parent_tree_id].root {
                self.shortest_path.push(node);
                node = self.tree_layers[parent_tree_id].parent[&node];
            }
        }
        self.shortest_path
            .push(*self.destination_nodes.last().unwrap());
        self.is_distance_matrix_updated = false;
    }

    fn solve_rtsp(&mut self, force: bool) {
        if !self.is_connected_graph {
            return;
        }
        if !force && !self.is_distance_matrix_updated {
            return;
        }

        if self.config.general.pseudo == 0 {
            let (path_cost, sequence) = self.eci_gen_solver.solve_rtsp(
                &self.distance_matrix,
                self.source_tree_id as usize,
                self.target_tree_id as usize,
            );
            if path_cost < self.shortest_path_cost && path_cost.is_finite() {
                self.sequence_of_tree_id_rtsp = sequence;
                self.shortest_path_cost = path_cost;
                self.update_path();
                if !self.shortest_path.is_empty() {
                    self.store_best_result();
                    self.log_data();
                }
            }
        } else {
            if !self.is_merge_done {
                self.merge_pseudo_trees();
                self.sequence_of_tree_id_rtsp =
                    vec![self.source_tree_id as usize, self.target_tree_id as usize];
            }
            self.shortest_path_cost =
                self.distance_matrix[self.source_tree_id as usize][self.target_tree_id as usize];
            self.update_path();
            if !self.shortest_path.is_empty() {
                self.store_best_result();
                self.log_data();
            }
        }
    }

    fn tree_size(&self) -> usize {
        self.tree_layers.iter().map(|t| t.parent.len()).sum()
    }

    fn log_data(&mut self) {
        let tree_size = self.tree_size();
        let cpu_time = self.start_time.elapsed().as_secs_f64();
        let path_cost = self.shortest_path_cost;
        if let Some(log) = &mut self.experiment_log {
            log.record(cpu_time, path_cost, tree_size);
        }
    }

    fn store_best_result(&mut self) {
        let explored_nodes: usize = self.tree_layers.iter().map(|t| t.parent.len()).sum();
        self.best = Some(PlanningResult {
            path: self.shortest_path.clone(),
            visit_order: self.sequence_of_tree_id_rtsp.clone(),
            cost: self.shortest_path_cost,
            explored_nodes,
            elapsed_secs: self.start_time.elapsed().as_secs_f64(),
        });
    }
}

impl AnytimePlanner for ImomdRrtStar {
    fn step(&mut self) -> Result<StepResult> {
        ImomdRrtStar::step(self)
    }

    fn best_solution(&self) -> Option<&PlanningResult> {
        ImomdRrtStar::best_solution(self)
    }

    fn is_finished(&self) -> bool {
        ImomdRrtStar::is_finished(self)
    }

    fn run_until(&mut self, deadline: Instant) -> Result<PlanningResult> {
        ImomdRrtStar::run_until(self, deadline)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::*;
    use crate::map::FakeMapLoader;
    use crate::map::MapLoader;

    fn test_config(max_iter: usize) -> AlgorithmConfig {
        AlgorithmConfig::from_yaml_str(&format!(
            r#"
general: {{ system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: {max_iter}, max_time: 60 }}
rrt_params: {{ goal_bias: 1.0, random_seed: 0 }}
destinations: {{ source_id: 0, objective_ids: [1], target_id: 2 }}
map: {{ type: -1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 0, ga: {{ random_seed: 0, mutation_iter: 10, population: 10, generation: 1 }} }}
"#
        ))
        .unwrap()
    }

    #[test]
    fn planner_init_trees_count() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1],
            target: 2,
        };
        let planner = ImomdRrtStar::new(graph, dest, test_config(10)).unwrap();
        assert_eq!(planner.init_trees().len(), 3);
    }

    #[test]
    fn planner_rejects_invalid_node() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![99],
            target: 2,
        };
        assert!(ImomdRrtStar::new(graph, dest, test_config(10)).is_err());
    }

    #[test]
    fn planner_rejects_duplicate_destinations() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1, 1],
            target: 2,
        };
        assert!(ImomdRrtStar::new(graph, dest, test_config(10)).is_err());
    }

    #[test]
    fn planner_finds_path_on_fake_map_1() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1],
            target: 2,
        };
        let mut planner = ImomdRrtStar::new(graph, dest, test_config(5000)).unwrap();
        let result = planner.run_for(Duration::from_secs(5)).unwrap();
        assert!(!result.path.is_empty());
        assert_eq!(*result.path.first().unwrap(), 0);
        assert_eq!(*result.path.last().unwrap(), 2);
        assert!(result.cost > 0.0 && result.cost.is_finite());
    }

    #[test]
    fn caller_deadline_pauses_instead_of_finishing_anytime_search() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1],
            target: 2,
        };
        let mut planner = ImomdRrtStar::new(graph, dest, test_config(1_000_000)).unwrap();
        // Install a known best result so a zero-length caller time slice can
        // return without advancing (or terminating) the algorithm state.
        planner.best = Some(PlanningResult {
            path: vec![0, 1, 2],
            visit_order: vec![0, 1, 2],
            cost: 1.0,
            explored_nodes: 3,
            elapsed_secs: 0.0,
        });
        planner.run_for(Duration::ZERO).unwrap();
        assert!(
            !planner.is_finished(),
            "time slice must not terminate the planner"
        );
    }

    #[test]
    fn step_reports_real_state_transitions_and_exhaustion() {
        let graph = Arc::new(FakeMapLoader::new(-1).load().unwrap());
        let dest = Destinations {
            source: 0,
            objectives: vec![1],
            target: 2,
        };
        let mut planner = ImomdRrtStar::new(graph, dest, test_config(10_000)).unwrap();
        let mut saw_transition = false;
        for _ in 0..1_000 {
            let step = planner.step().unwrap();
            saw_transition |= matches!(
                step.status,
                StepStatus::Connected | StepStatus::PathImproved
            );
            if planner.exploration_exhausted() && planner.best_solution().is_some() {
                break;
            }
            if planner.is_finished() {
                break;
            }
        }
        assert!(saw_transition, "expected a connection or path improvement");
        assert!(
            planner.exploration_exhausted() || planner.is_finished(),
            "finite fake graph should exhaust destination trees or hit limits"
        );
        assert!(planner.best_solution().is_some());
    }
}
