use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::config::AlgorithmConfig;
use crate::error::{PlannerError, Result};
use crate::graph::AdjacencyGraph;
use crate::navigation::events::{PlanUpdate, UpdateReason};
use crate::navigation::plugin::PlannerPlugin;
use crate::rrt::ImomdRrtStar;
use crate::types::{Destinations, GraphUpdateStats, NodeId, PlanningResult, StepStatus};

/// Adapter that exposes [`ImomdRrtStar`] as a [`PlannerPlugin`].
pub struct ImomdPlugin {
    config: AlgorithmConfig,
    graph: Option<Arc<AdjacencyGraph>>,
    inner: Option<ImomdRrtStar>,
    sequence: u64,
    replan_mode: String,
    last_tree_update: Option<GraphUpdateStats>,
    ego_node: Option<NodeId>,
}

impl ImomdPlugin {
    pub fn new(config: AlgorithmConfig) -> Self {
        Self {
            config,
            graph: None,
            inner: None,
            sequence: 0,
            replan_mode: "fresh".into(),
            last_tree_update: None,
            ego_node: None,
        }
    }

    pub fn with_default_config() -> Self {
        let config = AlgorithmConfig::from_yaml_str(
            r#"
general: { system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: 2000000, max_time: 3600 }
rrt_params: { goal_bias: 1.0, random_seed: 0 }
destinations: { source_id: 0, objective_ids: [1], target_id: 2 }
map: { type: -1, path: "", name: "" }
rtsp_settings: { shortcut: 1, swapping: 1, genetic: 1, ga: { random_seed: 0, mutation_iter: 20, population: 20, generation: 2 } }
"#,
        )
        .expect("default ImomdPlugin config must parse");
        Self::new(config)
    }

    fn next_seq(&mut self) -> u64 {
        self.sequence += 1;
        self.sequence
    }

    fn make_update(
        &mut self,
        reason: UpdateReason,
        best: Option<&PlanningResult>,
    ) -> PlanUpdate {
        let seq = self.next_seq();
        if let Some(best) = best {
            PlanUpdate::from_best(
                seq,
                reason,
                best,
                self.replan_mode.clone(),
                "imomd",
                self.last_tree_update,
                self.ego_node,
            )
        } else {
            PlanUpdate::marker(
                seq,
                reason,
                self.replan_mode.clone(),
                "imomd",
                self.last_tree_update,
                self.ego_node,
            )
        }
    }
}

impl PlannerPlugin for ImomdPlugin {
    fn id(&self) -> &'static str {
        "imomd"
    }

    fn reset(&mut self, graph: Arc<AdjacencyGraph>, destinations: Destinations) -> Result<()> {
        self.graph = Some(Arc::clone(&graph));
        self.inner = Some(ImomdRrtStar::new(graph, destinations, self.config.clone())?);
        self.replan_mode = "fresh".into();
        self.last_tree_update = None;
        Ok(())
    }

    fn on_graph_changed(&mut self, graph: Arc<AdjacencyGraph>) -> Result<GraphUpdateStats> {
        let planner = self
            .inner
            .as_mut()
            .ok_or_else(|| PlannerError::Config("plugin not reset".into()))?;
        let stats = planner.update_graph(Arc::clone(&graph))?;
        self.graph = Some(graph);
        self.replan_mode = "warm_start".into();
        self.last_tree_update = Some(stats);
        Ok(stats)
    }

    fn on_ego_moved(&mut self, ego_node: NodeId, remaining: Destinations) -> Result<()> {
        let graph = self
            .graph
            .clone()
            .ok_or_else(|| PlannerError::Config("plugin not reset".into()))?;
        let mut destinations = remaining;
        destinations.source = ego_node;
        self.ego_node = Some(ego_node);
        self.reset(graph, destinations)?;
        self.replan_mode = "ego_reseed".into();
        Ok(())
    }

    fn continue_search(&mut self, budget: Duration) -> Result<Vec<PlanUpdate>> {
        let deadline = Instant::now() + budget;
        let mut updates = Vec::new();
        let mut last_cost = self.best().map(|r| r.cost);
        self.replan_mode = "resume".into();

        // Anytime semantics: each budget slice must keep searching / refining
        // even if the previous slice hit expansion_finished / max_time.
        if let Some(planner) = self.inner.as_mut() {
            planner.resume_search();
        }

        loop {
            if Instant::now() >= deadline {
                break;
            }

            let step = {
                let planner = self
                    .inner
                    .as_mut()
                    .ok_or_else(|| PlannerError::Config("plugin not reset".into()))?;
                planner.step()?
            };

            if matches!(step.status, StepStatus::Finished) {
                // Early terminal inside a slice: reopen and spend remaining budget.
                if let Some(planner) = self.inner.as_mut() {
                    planner.resume_search();
                    planner.refine_solution(4);
                }
                let best = self.best().cloned();
                let improved = match (last_cost, best.as_ref().map(|b| b.cost)) {
                    (Some(old), Some(new)) if new + f64::EPSILON < old => true,
                    _ => false,
                };
                if improved {
                    last_cost = best.as_ref().map(|b| b.cost);
                    updates.push(self.make_update(UpdateReason::Improved, best.as_ref()));
                }
                continue;
            }

            let best = self.best().cloned();
            let improved = match (last_cost, best.as_ref().map(|b| b.cost)) {
                (Some(old), Some(new)) if new + f64::EPSILON < old => true,
                (None, Some(_)) => true,
                _ => false,
            };
            if improved {
                last_cost = best.as_ref().map(|b| b.cost);
            }

            let reason = if improved {
                UpdateReason::Improved
            } else {
                match step.status {
                    StepStatus::Expanded => UpdateReason::Expanded,
                    StepStatus::Connected => UpdateReason::Connected,
                    StepStatus::PathImproved => UpdateReason::Improved,
                    StepStatus::Finished => UpdateReason::Finished,
                }
            };

            if !matches!(reason, UpdateReason::Expanded) {
                updates.push(self.make_update(reason, best.as_ref()));
            }
        }

        if updates.is_empty() {
            if let Some(best) = self.best().cloned() {
                updates.push(self.make_update(UpdateReason::Resume, Some(&best)));
            }
        }
        Ok(updates)
    }

    fn best(&self) -> Option<&PlanningResult> {
        self.inner.as_ref().and_then(|p| p.best_solution())
    }

    fn is_finished(&self) -> bool {
        self.inner.as_ref().map(|p| p.is_finished()).unwrap_or(true)
    }
}
