use std::sync::Arc;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use crate::error::{PlannerError, Result};
use crate::geo::haversine_distance;
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::navigation::events::{DomainEvent, PlanUpdate, UpdateReason};
use crate::navigation::gate::BestCostGate;
use crate::navigation::plugin::PlannerPlugin;
use crate::navigation::tour::{solve_exact_tour, solve_greedy_tour};
use crate::types::{Destinations, NodeId, PlanningResult};

/// Owns destinations, ego pose, graph revision, and a pluggable planner.
///
/// On destination / traffic events, races greedy + exact (+ IMOMD) and admits
/// updates through [`BestCostGate`] (first feasible wins; only better cost covers).
pub struct NavigationSession {
    plugin: Box<dyn PlannerPlugin>,
    graph: Option<Arc<AdjacencyGraph>>,
    destinations: Option<Destinations>,
    ego_node: Option<NodeId>,
    graph_revision: u64,
    applied_revision: u64,
    sequence_offset: u64,
    gate: BestCostGate,
    /// Cached best from any lane (for `best()`).
    best: Option<PlanningResult>,
    best_algorithm: Option<String>,
    race_epoch: u64,
}

impl NavigationSession {
    pub fn new(plugin: Box<dyn PlannerPlugin>) -> Self {
        Self {
            plugin,
            graph: None,
            destinations: None,
            ego_node: None,
            graph_revision: 0,
            applied_revision: 0,
            sequence_offset: 0,
            gate: BestCostGate::new(),
            best: None,
            best_algorithm: None,
            race_epoch: 0,
        }
    }

    pub fn algorithm_id(&self) -> &str {
        self.best_algorithm
            .as_deref()
            .unwrap_or_else(|| self.plugin.id())
    }

    pub fn ego_node(&self) -> Option<NodeId> {
        self.ego_node
    }

    pub fn destinations(&self) -> Option<&Destinations> {
        self.destinations.as_ref()
    }

    pub fn best(&self) -> Option<&PlanningResult> {
        self.best.as_ref().or_else(|| self.plugin.best())
    }

    pub fn graph_revision(&self) -> u64 {
        self.graph_revision
    }

    /// Replace the active road graph snapshot (topology + weights).
    pub fn set_graph(&mut self, graph: Arc<AdjacencyGraph>) {
        self.graph = Some(graph);
        self.graph_revision = self.graph_revision.saturating_add(1);
    }

    /// Snap a geographic pose to the nearest graph node.
    pub fn snap_ego(&self, latitude: f64, longitude: f64) -> Result<NodeId> {
        let graph = self
            .graph
            .as_ref()
            .ok_or_else(|| PlannerError::Config("graph not set".into()))?;
        let mut best = None;
        for node in 0..graph.node_count() {
            let Some(loc) = graph.location(node) else {
                continue;
            };
            let d = haversine_distance(latitude, longitude, loc.latitude, loc.longitude);
            match best {
                Some((_, best_d)) if d >= best_d => {}
                _ => best = Some((node, d)),
            }
        }
        best.map(|(n, _)| n)
            .ok_or_else(|| PlannerError::Config("empty graph".into()))
    }

    /// Apply one domain event and return streamable plan updates.
    pub fn handle(&mut self, event: DomainEvent, budget: Duration) -> Result<Vec<PlanUpdate>> {
        match event {
            DomainEvent::DestinationsSet {
                source,
                objectives,
                target,
            } => {
                let graph = self.require_graph()?;
                let destinations = Destinations {
                    source,
                    objectives,
                    target,
                };
                self.ego_node = Some(source);
                self.destinations = Some(destinations.clone());
                self.race_epoch = self.race_epoch.saturating_add(1);
                self.gate.reset();
                self.best = None;
                self.best_algorithm = None;
                self.plugin.reset(Arc::clone(&graph), destinations.clone())?;
                self.applied_revision = self.graph_revision;

                let mut updates = vec![self.marker(UpdateReason::Fresh)];
                updates.extend(self.race_lanes(graph, destinations, budget, /*warm_imomd=*/ false)?);
                Ok(self.renumber(updates))
            }
            DomainEvent::TrafficChanged => {
                let graph = self.require_graph()?;
                if self.destinations.is_none() {
                    return Err(PlannerError::Config(
                        "destinations not set before traffic change".into(),
                    ));
                }
                let destinations = self.destinations.clone().unwrap();
                self.race_epoch = self.race_epoch.saturating_add(1);
                self.gate.invalidate_for_traffic();
                let stats = self.plugin.on_graph_changed(Arc::clone(&graph))?;
                self.applied_revision = self.graph_revision;
                let mut updates = vec![PlanUpdate::marker(
                    0,
                    UpdateReason::TrafficWarmStart,
                    "warm_start",
                    self.plugin.id(),
                    Some(stats),
                    self.ego_node,
                )];
                updates.extend(self.race_lanes(graph, destinations, budget, /*warm_imomd=*/ true)?);
                Ok(self.renumber(updates))
            }
            DomainEvent::EgoMoved { ego_node } => {
                let mut remaining = self
                    .destinations
                    .clone()
                    .ok_or_else(|| PlannerError::Config("destinations not set".into()))?;
                remaining.objectives.retain(|&n| n != ego_node);
                remaining.source = ego_node;
                self.ego_node = Some(ego_node);
                self.destinations = Some(remaining.clone());
                self.race_epoch = self.race_epoch.saturating_add(1);
                self.gate.reset();
                self.best = None;
                self.best_algorithm = None;
                let graph = self.require_graph()?;
                self.plugin.on_ego_moved(ego_node, remaining.clone())?;
                let mut updates = vec![self.marker(UpdateReason::EgoReseed)];
                updates.extend(self.race_lanes(graph, remaining, budget, false)?);
                Ok(self.renumber(updates))
            }
            DomainEvent::ContinueSearch => {
                if self.applied_revision != self.graph_revision {
                    return self.handle(DomainEvent::TrafficChanged, budget);
                }
                let raw = self.plugin.continue_search(budget)?;
                let mut admitted = self.admit_all(raw);
                // Anytime slice with no improvement: still emit a resume heartbeat
                // so clients know search ran (gate may reject equal-cost paths).
                if admitted.is_empty() {
                    if let Some(best) = self.best.clone() {
                        admitted.push(PlanUpdate::from_best(
                            0,
                            UpdateReason::Resume,
                            &best,
                            "resume",
                            self.algorithm_id().to_string(),
                            None,
                            self.ego_node,
                        ));
                    }
                }
                Ok(self.renumber(admitted))
            }
        }
    }

    /// Race greedy + exact in worker threads while IMOMD searches on this thread.
    fn race_lanes(
        &mut self,
        graph: Arc<AdjacencyGraph>,
        destinations: Destinations,
        budget: Duration,
        _warm_imomd: bool,
    ) -> Result<Vec<PlanUpdate>> {
        let (tx, rx) = mpsc::channel::<PlanUpdate>();

        {
            let g = Arc::clone(&graph);
            let d = destinations.clone();
            let tx = tx.clone();
            thread::spawn(move || {
                if let Some(sol) = solve_greedy_tour(&g, &d) {
                    let _ = tx.send(PlanUpdate::from_best(
                        0,
                        UpdateReason::GreedyInit,
                        &sol.result,
                        "race",
                        sol.algorithm_id,
                        None,
                        Some(d.source),
                    ));
                }
            });
        }
        {
            let g = Arc::clone(&graph);
            let d = destinations.clone();
            let tx = tx.clone();
            thread::spawn(move || {
                if let Some(sol) = solve_exact_tour(&g, &d) {
                    let _ = tx.send(PlanUpdate::from_best(
                        0,
                        UpdateReason::ExactOptimal,
                        &sol.result,
                        "race",
                        sol.algorithm_id,
                        None,
                        Some(d.source),
                    ));
                }
            });
        }
        drop(tx);

        let deadline = Instant::now() + budget;
        let mut admitted = Vec::new();

        // Interleave short IMOMD slices with draining finished race lanes.
        while Instant::now() < deadline {
            let slice = Duration::from_millis(40).min(deadline.saturating_duration_since(Instant::now()));
            if slice.is_zero() {
                break;
            }
            let raw = self.plugin.continue_search(slice)?;
            admitted.extend(self.admit_all(raw));

            while let Ok(update) = rx.try_recv() {
                if self.gate.admit(&update) {
                    self.note_best(&update);
                    admitted.push(update);
                }
            }

            if self.plugin.is_finished() {
                break;
            }
        }

        // Collect any straggler greedy/exact results (still gated).
        let wait = Duration::from_millis(500);
        let end = Instant::now() + wait;
        while Instant::now() < end {
            match rx.recv_timeout(Duration::from_millis(20)) {
                Ok(update) => {
                    if self.gate.admit(&update) {
                        self.note_best(&update);
                        admitted.push(update);
                    }
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        }
        while let Ok(update) = rx.try_recv() {
            if self.gate.admit(&update) {
                self.note_best(&update);
                admitted.push(update);
            }
        }

        Ok(admitted)
    }

    fn admit_all(&mut self, updates: Vec<PlanUpdate>) -> Vec<PlanUpdate> {
        let mut out = Vec::new();
        for update in updates {
            if self.gate.admit(&update) {
                self.note_best(&update);
                out.push(update);
            } else if update.path.is_none() {
                // Non-path markers already handled inside gate; if rejected, skip.
            }
        }
        out
    }

    fn note_best(&mut self, update: &PlanUpdate) {
        if let (Some(path), Some(cost)) = (&update.path, update.cost) {
            if path.is_empty() || !cost.is_finite() {
                return;
            }
            self.best = Some(PlanningResult {
                path: path.clone(),
                visit_order: update.visit_order.clone().unwrap_or_default(),
                cost,
                explored_nodes: update.explored_nodes.unwrap_or(0),
                elapsed_secs: 0.0,
            });
            self.best_algorithm = Some(update.algorithm_id.clone());
        }
    }

    fn require_graph(&self) -> Result<Arc<AdjacencyGraph>> {
        self.graph
            .clone()
            .ok_or_else(|| PlannerError::Config("graph not set".into()))
    }

    fn marker(&self, reason: UpdateReason) -> PlanUpdate {
        PlanUpdate::marker(
            0,
            reason,
            match reason {
                UpdateReason::Fresh => "fresh",
                UpdateReason::TrafficWarmStart => "warm_start",
                UpdateReason::EgoReseed => "ego_reseed",
                UpdateReason::Resume => "resume",
                UpdateReason::GreedyInit | UpdateReason::ExactOptimal => "race",
                _ => "resume",
            },
            self.plugin.id(),
            None,
            self.ego_node,
        )
    }

    fn renumber(&mut self, mut updates: Vec<PlanUpdate>) -> Vec<PlanUpdate> {
        for update in &mut updates {
            self.sequence_offset = self.sequence_offset.saturating_add(1);
            update.sequence = self.sequence_offset;
            update.ego_node = self.ego_node.or(update.ego_node);
        }
        updates
    }
}
