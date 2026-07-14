use std::cmp::Ordering;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use crate::rtsp::DistanceMatrix;

/// Brute-force TSP over destination trees with Dijkstra completion for disconnected pairs.
/// Maps to C++ `tsp_greed` in `greedy_tsp.h`.
pub fn solve_brute_force(
    distance_matrix: &DistanceMatrix,
    source_id: usize,
    target_id: usize,
) -> (f64, Vec<usize>) {
    let n = distance_matrix.len();
    let objectives: Vec<usize> = (0..n)
        .filter(|&i| i != source_id && i != target_id)
        .collect();

    let mut complete = distance_matrix.clone();
    let mut waypoints: HashMap<(usize, usize), Vec<usize>> = HashMap::new();

    for (i, row) in complete.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            if cell.is_infinite() {
                let (cost, path) = find_connection_dijkstra(distance_matrix, i, j);
                *cell = cost;
                if !path.is_empty() {
                    waypoints.insert((i, j), path);
                }
            }
        }
    }

    let mut min_cost = f64::INFINITY;
    let mut min_path: Vec<usize> = Vec::new();
    let mut all_options: Vec<Vec<usize>> = Vec::new();
    nested_loop(&objectives, &[], &mut all_options);

    for option in &all_options {
        let mut tmp_cost = complete[source_id][option[0]];
        for i in 0..option.len().saturating_sub(1) {
            tmp_cost += complete[option[i]][option[i + 1]];
        }
        if !option.is_empty() {
            tmp_cost += complete[option[option.len() - 1]][target_id];
        } else {
            tmp_cost = complete[source_id][target_id];
        }

        if tmp_cost < min_cost {
            min_cost = tmp_cost;
            min_path = option.clone();
        }
    }

    let mut full_path = min_path;
    full_path.insert(0, source_id);
    full_path.push(target_id);

    let mut shortest_path = Vec::new();
    for i in 0..full_path.len() - 1 {
        shortest_path.push(full_path[i]);
        if let Some(wp) = waypoints.get(&(full_path[i], full_path[i + 1])) {
            shortest_path.extend(wp.iter().copied());
        }
    }
    shortest_path.push(target_id);

    (min_cost, shortest_path)
}

fn nested_loop(objectives: &[usize], path: &[usize], all_options: &mut Vec<Vec<usize>>) {
    if objectives.len() > 1 {
        for i in 0..objectives.len() {
            let obj = objectives[i];
            let mut tmp_objectives = objectives.to_vec();
            tmp_objectives.remove(i);
            let mut tmp_path = path.to_vec();
            tmp_path.push(obj);
            nested_loop(&tmp_objectives, &tmp_path, all_options);
        }
    } else if let Some(&obj) = objectives.first() {
        let mut final_path = path.to_vec();
        final_path.push(obj);
        all_options.push(final_path);
    } else {
        all_options.push(path.to_vec());
    }
}

#[derive(Eq, PartialEq)]
struct DijkstraState {
    cost: OrderedFloat,
    node: usize,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct OrderedFloat(u64);

impl OrderedFloat {
    fn new(v: f64) -> Self {
        Self(v.to_bits())
    }
}

impl PartialOrd for DijkstraState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for DijkstraState {
    fn cmp(&self, other: &Self) -> Ordering {
        self.cost.cmp(&other.cost)
    }
}

fn find_connection_dijkstra(
    distance_matrix: &DistanceMatrix,
    source: usize,
    target: usize,
) -> (f64, Vec<usize>) {
    if !distance_matrix[source][target].is_infinite() {
        return (distance_matrix[source][target], Vec::new());
    }

    let mut parent: HashMap<usize, usize> = HashMap::from([(source, source)]);
    let mut cost_from_source: HashMap<usize, f64> =
        HashMap::from([(source, 0.0), (target, f64::INFINITY)]);

    let mut open_queue = BinaryHeap::new();
    open_queue.push(Reverse(DijkstraState {
        cost: OrderedFloat::new(0.0),
        node: source,
    }));

    while let Some(Reverse(state)) = open_queue.pop() {
        let current_cost = cost_from_source
            .get(&state.node)
            .copied()
            .unwrap_or(f64::INFINITY);
        if OrderedFloat::new(current_cost) != state.cost {
            continue;
        }
        if state.node == target {
            break;
        }

        for (i, &edge) in distance_matrix[state.node].iter().enumerate() {
            if state.node == i {
                continue;
            }
            if edge.is_infinite() {
                continue;
            }

            let new_cost = cost_from_source[&state.node] + edge;
            let entry = cost_from_source.entry(i).or_insert(f64::INFINITY);
            if new_cost < *entry {
                *entry = new_cost;
                parent.insert(i, state.node);
                open_queue.push(Reverse(DijkstraState {
                    cost: OrderedFloat::new(new_cost),
                    node: i,
                }));
            }
        }
    }

    let mut waypoints = Vec::new();
    let mut state = parent.get(&target).copied().unwrap_or(source);
    while state != source {
        waypoints.push(state);
        state = parent[&state];
    }
    waypoints.reverse();

    (cost_from_source[&target], waypoints)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brute_force_three_node_chain() {
        let matrix = vec![
            vec![0.0, 1.0, 3.0],
            vec![1.0, 0.0, 1.0],
            vec![3.0, 1.0, 0.0],
        ];
        let (cost, seq) = solve_brute_force(&matrix, 0, 2);
        assert_eq!(seq, vec![0, 1, 2]);
        assert!((cost - 2.0).abs() < 1e-9);
    }

    #[test]
    fn dijkstra_fills_infinite_gap() {
        let matrix = vec![
            vec![0.0, 1.0, f64::INFINITY],
            vec![1.0, 0.0, 1.0],
            vec![f64::INFINITY, 1.0, 0.0],
        ];
        let (cost, waypoints) = find_connection_dijkstra(&matrix, 0, 2);
        assert_eq!(waypoints, vec![1]);
        assert!((cost - 2.0).abs() < 1e-9);
    }
}
