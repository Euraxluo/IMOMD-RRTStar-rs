//! Multi-objective tour solvers for the race lanes (greedy + exact).

use std::sync::Arc;
use std::time::Instant;

use rayon::prelude::*;

use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::navigation::shortest::{dijkstra_path, stitch_legs};
use crate::types::{Destinations, NodeId, PlanningResult};

/// Max middle objectives for exact permutation TSP.
pub const EXACT_MAX_OBJECTIVES: usize = 8;

#[derive(Debug, Clone)]
pub struct TourSolution {
    pub result: PlanningResult,
    pub algorithm_id: &'static str,
}

fn terminals(dest: &Destinations) -> Vec<NodeId> {
    dest.all_nodes()
}

/// Pairwise Dijkstra among terminals. `dist[i][j]` / `paths[i][j]` use terminal indices.
fn pairwise_dijkstra(
    graph: &AdjacencyGraph,
    terminals: &[NodeId],
) -> Option<(Vec<Vec<f64>>, Vec<Vec<Vec<NodeId>>>)> {
    let t = terminals.len();
    let mut dist = vec![vec![f64::INFINITY; t]; t];
    let mut paths = vec![vec![Vec::new(); t]; t];

    let pairs: Vec<(usize, usize)> = (0..t)
        .flat_map(|i| (0..t).filter(move |&j| i != j).map(move |j| (i, j)))
        .collect();

    let computed: Vec<Option<(usize, usize, f64, Vec<NodeId>)>> = pairs
        .par_iter()
        .map(|&(i, j)| {
            dijkstra_path(graph as &dyn RoadGraph, terminals[i], terminals[j])
                .map(|(c, p)| (i, j, c, p))
        })
        .collect();

    for entry in computed {
        let (i, j, c, p) = entry?;
        dist[i][j] = c;
        paths[i][j] = p;
    }
    Some((dist, paths))
}

fn materialize_tour(
    order_terminal_idx: &[usize],
    paths: &[Vec<Vec<NodeId>>],
    dist: &[Vec<f64>],
    explored: usize,
    elapsed: f64,
) -> Option<PlanningResult> {
    if order_terminal_idx.len() < 2 {
        return None;
    }
    let mut cost = 0.0;
    let mut legs = Vec::new();
    for window in order_terminal_idx.windows(2) {
        let a = window[0];
        let b = window[1];
        let c = dist[a][b];
        if !c.is_finite() {
            return None;
        }
        cost += c;
        let leg = &paths[a][b];
        if leg.is_empty() {
            return None;
        }
        legs.push(leg.clone());
    }
    Some(PlanningResult {
        path: stitch_legs(&legs),
        visit_order: order_terminal_idx.to_vec(),
        cost,
        explored_nodes: explored,
        elapsed_secs: elapsed,
    })
}

/// Nearest-neighbor order on the terminal distance matrix (fixed source/target).
pub fn solve_greedy_tour(
    graph: &Arc<AdjacencyGraph>,
    dest: &Destinations,
) -> Option<TourSolution> {
    let start = Instant::now();
    let terms = terminals(dest);
    let t = terms.len();
    if t < 2 {
        return None;
    }
    let (dist, paths) = pairwise_dijkstra(graph.as_ref(), &terms)?;
    let source = 0usize;
    let target = t - 1;
    let mut remaining: Vec<usize> = (1..target).collect();
    let mut order = vec![source];
    let mut cur = source;
    while !remaining.is_empty() {
        let mut best_i = 0;
        let mut best_d = f64::INFINITY;
        for (i, &node) in remaining.iter().enumerate() {
            let d = dist[cur][node];
            if d < best_d {
                best_d = d;
                best_i = i;
            }
        }
        if !best_d.is_finite() {
            return None;
        }
        let next = remaining.remove(best_i);
        order.push(next);
        cur = next;
    }
    order.push(target);
    let result = materialize_tour(
        &order,
        &paths,
        &dist,
        terms.len(),
        start.elapsed().as_secs_f64(),
    )?;
    Some(TourSolution {
        result,
        algorithm_id: "greedy",
    })
}

/// Exact tour: pairwise Dijkstra + objective permutation enumeration.
pub fn solve_exact_tour(
    graph: &Arc<AdjacencyGraph>,
    dest: &Destinations,
) -> Option<TourSolution> {
    let start = Instant::now();
    if dest.objectives.len() > EXACT_MAX_OBJECTIVES {
        return None;
    }
    let terms = terminals(dest);
    let t = terms.len();
    if t < 2 {
        return None;
    }
    let (dist, paths) = pairwise_dijkstra(graph.as_ref(), &terms)?;
    let source = 0usize;
    let target = t - 1;
    let middles: Vec<usize> = (1..target).collect();

    let mut best_order: Option<Vec<usize>> = None;
    let mut best_cost = f64::INFINITY;

    fn dfs(
        middles: &[usize],
        used: &mut [bool],
        prefix: &mut Vec<usize>,
        dist: &[Vec<f64>],
        source: usize,
        target: usize,
        best_cost: &mut f64,
        best_order: &mut Option<Vec<usize>>,
    ) {
        if prefix.len() == middles.len() {
            let mut order = vec![source];
            order.extend_from_slice(prefix);
            order.push(target);
            let mut cost = 0.0;
            for w in order.windows(2) {
                let c = dist[w[0]][w[1]];
                if !c.is_finite() {
                    return;
                }
                cost += c;
            }
            if cost < *best_cost {
                *best_cost = cost;
                *best_order = Some(order);
            }
            return;
        }
        for i in 0..middles.len() {
            if used[i] {
                continue;
            }
            used[i] = true;
            prefix.push(middles[i]);
            dfs(
                middles, used, prefix, dist, source, target, best_cost, best_order,
            );
            prefix.pop();
            used[i] = false;
        }
    }

    if middles.is_empty() {
        best_order = Some(vec![source, target]);
        best_cost = dist[source][target];
    } else {
        let mut used = vec![false; middles.len()];
        let mut prefix = Vec::new();
        dfs(
            &middles,
            &mut used,
            &mut prefix,
            &dist,
            source,
            target,
            &mut best_cost,
            &mut best_order,
        );
    }

    let order = best_order?;
    if !best_cost.is_finite() {
        return None;
    }
    let result = materialize_tour(
        &order,
        &paths,
        &dist,
        terms.len(),
        start.elapsed().as_secs_f64(),
    )?;
    Some(TourSolution {
        result,
        algorithm_id: "exact",
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Location;
    use rustc_hash::FxHashMap;

    fn square() -> Arc<AdjacencyGraph> {
        // 0-1-3
        // |/
        // 2
        let nodes = (0..4)
            .map(|i| Location::new(i, 0.0, i as f64))
            .collect();
        let mut edges = vec![FxHashMap::default(); 4];
        let add = |e: &mut [FxHashMap<NodeId, f64>], a: usize, b: usize, w: f64| {
            e[a].insert(b, w);
            e[b].insert(a, w);
        };
        add(&mut edges, 0, 1, 1.0);
        add(&mut edges, 1, 3, 1.0);
        add(&mut edges, 0, 2, 1.0);
        add(&mut edges, 2, 3, 5.0);
        add(&mut edges, 1, 2, 1.0);
        Arc::new(AdjacencyGraph::new(nodes, edges).unwrap())
    }

    #[test]
    fn exact_beats_naive_order_when_needed() {
        let g = square();
        let dest = Destinations {
            source: 0,
            objectives: vec![2],
            target: 3,
        };
        let exact = solve_exact_tour(&g, &dest).unwrap();
        // 0-1-2-1-3 would be worse; optimal 0-2 via? 0-1-2 then 2-1-3 = 1+1+1+1=4
        // or 0-2, 2-1-3 = 1+1+1=3
        assert!(exact.result.cost <= 4.0 + 1e-9);
        assert!(exact.result.path.contains(&2));
    }
}
