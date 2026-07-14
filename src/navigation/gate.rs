//! Admit plan updates only when they improve (or are the first feasible path).

use crate::navigation::events::PlanUpdate;

const COST_EPS: f64 = 1e-6;

#[derive(Debug, Default)]
pub struct BestCostGate {
    best_cost: Option<f64>,
    /// After traffic change, allow the next feasible path even if worse than
    /// a stale pre-traffic best.
    allow_first_after_invalidate: bool,
}

impl BestCostGate {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.best_cost = None;
        self.allow_first_after_invalidate = false;
    }

    /// Traffic epoch: drop trust in previous best; next feasible may show.
    pub fn invalidate_for_traffic(&mut self) {
        self.best_cost = None;
        self.allow_first_after_invalidate = true;
    }

    /// Returns true if `update` should be published to clients.
    pub fn admit(&mut self, update: &PlanUpdate) -> bool {
        let Some(cost) = update.cost.filter(|c| c.is_finite()) else {
            // Markers (fresh / warm_start) always pass through.
            return update.path.is_none();
        };
        if update.path.as_ref().is_none_or(|p| p.is_empty()) {
            return false;
        }

        match self.best_cost {
            None => {
                self.best_cost = Some(cost);
                self.allow_first_after_invalidate = false;
                true
            }
            Some(best) if cost < best - COST_EPS => {
                self.best_cost = Some(cost);
                true
            }
            Some(_) => false,
        }
    }

    pub fn best_cost(&self) -> Option<f64> {
        self.best_cost
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::UpdateReason;

    fn path_update(cost: f64) -> PlanUpdate {
        PlanUpdate {
            sequence: 0,
            reason: UpdateReason::Improved,
            path: Some(vec![0, 1, 2]),
            cost: Some(cost),
            visit_order: None,
            explored_nodes: None,
            replan_mode: "race".into(),
            tree_update: None,
            ego_node: None,
            algorithm_id: "greedy".into(),
        }
    }

    #[test]
    fn first_then_only_better() {
        let mut g = BestCostGate::new();
        assert!(g.admit(&path_update(100.0)));
        assert!(!g.admit(&path_update(100.0)));
        assert!(!g.admit(&path_update(110.0)));
        assert!(g.admit(&path_update(90.0)));
    }

    #[test]
    fn traffic_invalidates() {
        let mut g = BestCostGate::new();
        assert!(g.admit(&path_update(50.0)));
        g.invalidate_for_traffic();
        assert!(g.admit(&path_update(80.0)));
    }
}
