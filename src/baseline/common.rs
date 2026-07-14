use std::cmp::Ordering;
use std::sync::Arc;
use std::time::Instant;

use rand::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::config::AlgorithmConfig;
use crate::error::{PlannerError, Result};
use crate::experiment::{ExperimentLog, ExperimentRecord};
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::rtsp::EciGenSolver;
use crate::types::{NodeId, PlanningResult};

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct OrderedFloat(u64);

impl OrderedFloat {
    pub(crate) fn new(v: f64) -> Self {
        Self(v.to_bits())
    }
}

#[derive(Eq, PartialEq)]
pub(crate) struct PriorityState {
    pub(crate) priority: OrderedFloat,
    pub(crate) node: NodeId,
}

impl PartialOrd for PriorityState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PriorityState {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority.cmp(&other.priority)
    }
}

pub(crate) fn pair_tree_idx(start_id: i32, goal_id: i32) -> usize {
    if start_id > goal_id {
        (start_id * (start_id - 1) / 2 + goal_id) as usize
    } else {
        (goal_id * (goal_id - 1) / 2 + start_id) as usize
    }
}

/// Shared destination-pair distance matrix state for baseline planners.
pub(crate) struct BaselineCore {
    pub graph: Arc<AdjacencyGraph>,
    pub config: AlgorithmConfig,
    pub destinations: Vec<NodeId>,
    pub source_tree_id: i32,
    pub target_tree_id: i32,
    pub distance_matrix: Vec<Vec<f64>>,
    pub connection_node_matrix: Vec<Vec<Option<NodeId>>>,
    pub disjoint_set_parent: Vec<i32>,
    pub disjoint_set_children: FxHashMap<i32, Vec<i32>>,
    pub is_connected_graph: bool,
    pub is_distance_matrix_updated: bool,
    pub eci_gen_solver: EciGenSolver,
    pub sequence_rtsp: Vec<usize>,
    pub shortest_path_cost: f64,
    pub shortest_path: Vec<NodeId>,
    pub iteration: usize,
    pub rng: StdRng,
    pub start_time: Instant,
    pub experiment_log: Option<ExperimentLog>,
}

impl BaselineCore {
    pub fn new(
        graph: Arc<AdjacencyGraph>,
        config: AlgorithmConfig,
        source: NodeId,
        objectives: Vec<NodeId>,
        target: NodeId,
    ) -> Result<Self> {
        config.validate()?;
        let mut unique_destinations = FxHashSet::default();
        for &node in std::iter::once(&source)
            .chain(objectives.iter())
            .chain(std::iter::once(&target))
        {
            if graph.location(node).is_none() {
                return Err(PlannerError::NodeNotFound(node));
            }
            if !unique_destinations.insert(node) {
                return Err(PlannerError::Config(format!(
                    "duplicate destination node: {node}"
                )));
            }
        }

        let mut destinations = vec![source];
        destinations.extend_from_slice(&objectives);
        destinations.push(target);

        let dest_count = destinations.len();
        let mut distance_matrix = vec![vec![f64::INFINITY; dest_count]; dest_count];
        let connection_node_matrix = vec![vec![None; dest_count]; dest_count];
        for (index, row) in distance_matrix.iter_mut().enumerate() {
            row[index] = 0.0;
        }

        let rtsp_settings = config.rtsp_settings.clone();
        let seed = if config.rrt_params.random_seed != 0 {
            rand::thread_rng().gen()
        } else {
            0
        };
        let log_name = match config.general.system {
            1 => "experiments/bi_astar_latest.csv",
            2 => "experiments/ana_star_latest.csv",
            _ => "experiments/baseline_latest.csv",
        };
        let experiment_log = ExperimentLog::from_enabled(
            config.general.log_data != 0,
            Some(std::path::PathBuf::from(log_name)),
        )?;

        Ok(Self {
            graph,
            config,
            destinations,
            source_tree_id: 0,
            target_tree_id: (dest_count - 1) as i32,
            distance_matrix,
            connection_node_matrix,
            disjoint_set_parent: (0..dest_count as i32).collect(),
            disjoint_set_children: FxHashMap::default(),
            is_connected_graph: false,
            is_distance_matrix_updated: false,
            eci_gen_solver: EciGenSolver::new(&rtsp_settings),
            sequence_rtsp: vec![0, dest_count - 1],
            shortest_path_cost: f64::INFINITY,
            shortest_path: Vec::new(),
            iteration: 0,
            rng: StdRng::seed_from_u64(seed),
            start_time: Instant::now(),
            experiment_log,
        })
    }

    pub fn heuristic(&self, from: NodeId, to: NodeId) -> f64 {
        self.graph.haversine(from, to).unwrap_or(f64::INFINITY)
    }

    pub fn connect_two_tree(&mut self, tree1_id: i32, tree2_id: i32) {
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

    /// Returns `true` when a strictly shorter RTSP tour was found.
    pub fn solve_rtsp(&mut self, force: bool) -> bool {
        if !self.is_connected_graph {
            return false;
        }
        if !force && !self.is_distance_matrix_updated {
            return false;
        }

        let (path_cost, sequence) = self.eci_gen_solver.solve_rtsp(
            &self.distance_matrix,
            self.source_tree_id as usize,
            self.target_tree_id as usize,
        );

        if path_cost < self.shortest_path_cost && path_cost.is_finite() {
            self.sequence_rtsp = sequence;
            self.shortest_path_cost = path_cost;
            self.is_distance_matrix_updated = false;
            true
        } else {
            false
        }
    }

    pub fn timed_out(&self) -> bool {
        self.iteration > self.config.general.max_iter
            || self.start_time.elapsed().as_secs() >= self.config.general.max_time
    }

    pub fn make_result(&self, explored_nodes: usize) -> Result<PlanningResult> {
        if self.shortest_path.is_empty() || !self.shortest_path_cost.is_finite() {
            return Err(PlannerError::Disconnected(
                self.destinations[0],
                *self.destinations.last().unwrap(),
            ));
        }
        Ok(PlanningResult {
            path: self.shortest_path.clone(),
            visit_order: self.sequence_rtsp.clone(),
            cost: self.shortest_path_cost,
            explored_nodes,
            elapsed_secs: self.start_time.elapsed().as_secs_f64(),
        })
    }

    pub fn log_data(&mut self, explored_nodes: usize) {
        if let Some(log) = &mut self.experiment_log {
            log.record(
                self.start_time.elapsed().as_secs_f64(),
                self.shortest_path_cost,
                explored_nodes,
            );
        }
    }

    pub fn experiment_records(&self) -> &[ExperimentRecord] {
        self.experiment_log
            .as_ref()
            .map(|log| log.records())
            .unwrap_or(&[])
    }
}
