use std::cmp::Ordering;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use rand::prelude::*;

use crate::config::RtspSettings;
use crate::rtsp::{DistanceMatrix, RtspSolution, RtspSolver};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InsertionType {
    Revisit,
    Sequence,
    SequenceSwapLeft,
    SequenceSwapRight,
    SequenceSwapBoth,
}

/// Enhanced Cheapest Insertion + Genetic Algorithm RTSP solver.
/// Maps to C++ `EciGenSolver` in `eci_gen_tsp_solver.h/.cpp`.
pub struct EciGenSolver {
    settings: RtspSettings,
    rng: StdRng,
    distance_matrix: DistanceMatrix,
    source_id: usize,
    target_id: usize,
    min_path_cost: f64,
    sequence_rtsp: Vec<usize>,
    sequence_best_insertion: VecDeque<usize>,
    sequence_dijkstra: VecDeque<usize>,
    ga_fitness: Vec<f64>,
    chromosomes: Vec<Vec<usize>>,
}

impl EciGenSolver {
    pub fn new(settings: &RtspSettings) -> Self {
        let seed = if settings.ga.random_seed != 0 {
            rand::thread_rng().gen()
        } else {
            0
        };
        Self {
            settings: settings.clone(),
            rng: StdRng::seed_from_u64(seed),
            distance_matrix: Vec::new(),
            source_id: 0,
            target_id: 0,
            min_path_cost: f64::INFINITY,
            sequence_rtsp: Vec::new(),
            sequence_best_insertion: VecDeque::new(),
            sequence_dijkstra: VecDeque::new(),
            ga_fitness: Vec::new(),
            chromosomes: Vec::new(),
        }
    }

    /// Solve relaxed TSP over the distance matrix (maps to `solveRTSP`).
    pub fn solve_rtsp(
        &mut self,
        distance_matrix: &DistanceMatrix,
        source_id: usize,
        target_id: usize,
    ) -> (f64, Vec<usize>) {
        self.distance_matrix = distance_matrix.clone();
        self.source_id = source_id;
        self.target_id = target_id;
        self.min_path_cost = 0.0;
        self.sequence_rtsp.clear();
        self.sequence_best_insertion.clear();
        self.sequence_dijkstra.clear();
        self.chromosomes.clear();
        self.ga_fitness.clear();

        self.solve_best_insertion();

        if self.settings.genetic != 0 {
            self.solve_genetic_algorithm();
        }

        (self.min_path_cost, self.sequence_rtsp.clone())
    }

    /// Solve source→target shortest path only (maps to `solveDijkstra`).
    pub fn solve_dijkstra(
        &mut self,
        distance_matrix: &DistanceMatrix,
        source_id: usize,
        target_id: usize,
    ) -> (f64, Vec<usize>) {
        self.distance_matrix = distance_matrix.clone();
        self.source_id = source_id;
        self.target_id = target_id;
        self.sequence_rtsp.clear();
        self.sequence_best_insertion.clear();
        self.sequence_dijkstra.clear();
        self.chromosomes.clear();
        self.ga_fitness.clear();

        self.min_path_cost = self.solve_dijkstra_internal();
        (
            self.min_path_cost,
            self.sequence_dijkstra.iter().copied().collect(),
        )
    }

    fn solve_best_insertion(&mut self) {
        let path_cost = self.solve_dijkstra_internal();
        self.sequence_best_insertion = self.sequence_dijkstra.clone();

        let n = self.distance_matrix.len();
        let mut not_visited: HashSet<usize> = (0..n).collect();
        for &visited in &self.sequence_dijkstra {
            not_visited.remove(&visited);
        }

        let mut total_cost = path_cost;
        while !not_visited.is_empty() {
            let mut min_insert_cost = f64::INFINITY;
            let mut left_idx_insert = 0;
            let mut insertion_method = InsertionType::Revisit;
            let mut id_insert = 0;

            for &tmp_insert_id in &not_visited {
                let (cost, (left_idx, method)) = self.best_insertion(tmp_insert_id);
                if cost < min_insert_cost {
                    id_insert = tmp_insert_id;
                    left_idx_insert = left_idx;
                    insertion_method = method;
                    min_insert_cost = cost;
                }
            }

            match insertion_method {
                InsertionType::Revisit => {
                    self.insert_revisit(id_insert, left_idx_insert);
                }
                InsertionType::Sequence => {
                    self.insert_sequence(id_insert, left_idx_insert);
                }
                InsertionType::SequenceSwapLeft => {
                    self.insert_sequence_swap_left(id_insert, left_idx_insert);
                }
                InsertionType::SequenceSwapRight => {
                    self.insert_sequence_swap_right(id_insert, left_idx_insert);
                }
                InsertionType::SequenceSwapBoth => {
                    self.insert_sequence_swap_both(id_insert, left_idx_insert);
                }
            }
            total_cost += min_insert_cost;
            not_visited.remove(&id_insert);
        }

        self.min_path_cost = total_cost;
        self.sequence_rtsp = self.sequence_best_insertion.iter().copied().collect();
        let mut seq = self.sequence_rtsp.clone();
        self.apply_shortcut(&mut seq);
        self.sequence_rtsp = seq;
    }

    fn solve_dijkstra_internal(&mut self) -> f64 {
        let n = self.distance_matrix.len();
        let mut parent: HashMap<usize, usize> = HashMap::from([(self.source_id, self.source_id)]);
        let mut cost_from_source: HashMap<usize, f64> =
            HashMap::from([(self.source_id, 0.0), (self.target_id, f64::INFINITY)]);

        #[derive(Eq, PartialEq)]
        struct State {
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
        impl PartialOrd for State {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                Some(self.cmp(other))
            }
        }
        impl Ord for State {
            fn cmp(&self, other: &Self) -> Ordering {
                self.cost.cmp(&other.cost)
            }
        }

        let mut open_queue = BinaryHeap::new();
        open_queue.push(Reverse(State {
            cost: OrderedFloat::new(0.0),
            node: self.source_id,
        }));

        while let Some(Reverse(state)) = open_queue.pop() {
            let current_cost = cost_from_source
                .get(&state.node)
                .copied()
                .unwrap_or(f64::INFINITY);
            if OrderedFloat::new(current_cost) != state.cost {
                continue;
            }
            if state.node == self.target_id {
                break;
            }
            for i in 0..n {
                if state.node == i {
                    continue;
                }
                let new_cost = cost_from_source[&state.node] + self.distance_matrix[state.node][i];
                let entry = cost_from_source.entry(i).or_insert(f64::INFINITY);
                if new_cost < *entry {
                    *entry = new_cost;
                    parent.insert(i, state.node);
                    open_queue.push(Reverse(State {
                        cost: OrderedFloat::new(new_cost),
                        node: i,
                    }));
                }
            }
        }

        self.extract_sequence(&parent, self.source_id, self.target_id);
        cost_from_source[&self.target_id]
    }

    fn extract_sequence(&mut self, parent: &HashMap<usize, usize>, source: usize, target: usize) {
        self.sequence_dijkstra.clear();
        self.sequence_dijkstra.push_back(target);
        let mut state = target;
        while state != source {
            state = parent[&state];
            self.sequence_dijkstra.push_back(state);
        }
        let mut seq: Vec<_> = self.sequence_dijkstra.iter().copied().collect();
        seq.reverse();
        self.sequence_dijkstra = seq.into();
    }

    fn best_insertion(&self, insert_id: usize) -> (f64, (usize, InsertionType)) {
        let mut min_insert_cost = f64::INFINITY;
        let mut best_left_idx = 0;
        let mut insertion_method = InsertionType::Revisit;

        for (left_idx, &left_val) in self.sequence_best_insertion.iter().enumerate() {
            let revisit_cost = 2.0 * self.distance_matrix[insert_id][left_val];
            let sequence_cost;
            let mut sequence_swap_left_cost = f64::INFINITY;
            let mut sequence_swap_right_cost = f64::INFINITY;
            let mut sequence_swap_both_cost = f64::INFINITY;

            if left_idx + 1 == self.sequence_best_insertion.len() {
                sequence_cost = revisit_cost;
            } else {
                let right_val = self.sequence_best_insertion[left_idx + 1];
                sequence_cost = self.distance_matrix[left_val][insert_id]
                    + self.distance_matrix[insert_id][right_val]
                    - self.distance_matrix[left_val][right_val];

                if self.settings.swapping != 0 {
                    if left_idx >= 2 {
                        let pv = self.sequence_best_insertion[left_idx - 1];
                        let ppv = self.sequence_best_insertion[left_idx - 2];
                        sequence_swap_left_cost = self.distance_matrix[insert_id][right_val]
                            - self.distance_matrix[left_val][right_val]
                            + self.distance_matrix[pv][insert_id]
                            + self.distance_matrix[ppv][left_val]
                            - self.distance_matrix[ppv][pv];
                    }
                    if left_idx + 3 < self.sequence_best_insertion.len() {
                        let nnx = self.sequence_best_insertion[left_idx + 2];
                        let nnnx = self.sequence_best_insertion[left_idx + 3];
                        sequence_swap_right_cost = self.distance_matrix[left_val][insert_id]
                            - self.distance_matrix[left_val][right_val]
                            + self.distance_matrix[nnx][insert_id]
                            + self.distance_matrix[nnnx][right_val]
                            - self.distance_matrix[nnnx][nnx];
                    }
                    if left_idx >= 2 && left_idx + 3 < self.sequence_best_insertion.len() {
                        let pv = self.sequence_best_insertion[left_idx - 1];
                        let ppv = self.sequence_best_insertion[left_idx - 2];
                        let nnx = self.sequence_best_insertion[left_idx + 2];
                        let nnnx = self.sequence_best_insertion[left_idx + 3];
                        sequence_swap_both_cost = self.distance_matrix[pv][insert_id]
                            + self.distance_matrix[ppv][left_val]
                            - self.distance_matrix[ppv][pv]
                            - self.distance_matrix[left_val][right_val]
                            + self.distance_matrix[nnx][insert_id]
                            + self.distance_matrix[nnnx][right_val]
                            - self.distance_matrix[nnnx][nnx];
                    }
                }
            }

            if revisit_cost < min_insert_cost {
                best_left_idx = left_idx;
                min_insert_cost = revisit_cost;
                insertion_method = InsertionType::Revisit;
            }
            if sequence_cost < min_insert_cost {
                best_left_idx = left_idx;
                min_insert_cost = sequence_cost;
                insertion_method = InsertionType::Sequence;
            }
            if self.settings.swapping != 0 {
                if sequence_swap_left_cost < min_insert_cost {
                    best_left_idx = left_idx;
                    min_insert_cost = sequence_swap_left_cost;
                    insertion_method = InsertionType::SequenceSwapLeft;
                }
                if sequence_swap_right_cost < min_insert_cost {
                    best_left_idx = left_idx;
                    min_insert_cost = sequence_swap_right_cost;
                    insertion_method = InsertionType::SequenceSwapRight;
                }
                if sequence_swap_both_cost < min_insert_cost {
                    best_left_idx = left_idx;
                    min_insert_cost = sequence_swap_both_cost;
                    insertion_method = InsertionType::SequenceSwapBoth;
                }
            }
        }

        (min_insert_cost, (best_left_idx, insertion_method))
    }

    fn insert_revisit(&mut self, value: usize, left_idx: usize) {
        let it_val = self.sequence_best_insertion[left_idx];
        self.sequence_best_insertion.insert(left_idx, value);
        self.sequence_best_insertion.insert(left_idx, it_val);
    }

    fn insert_sequence(&mut self, value: usize, left_idx: usize) {
        self.sequence_best_insertion.insert(left_idx + 1, value);
    }

    fn insert_sequence_swap_left(&mut self, value: usize, left_idx: usize) {
        self.sequence_best_insertion.swap(left_idx, left_idx - 1);
        self.sequence_best_insertion.insert(left_idx + 1, value);
    }

    fn insert_sequence_swap_right(&mut self, value: usize, left_idx: usize) {
        self.sequence_best_insertion
            .swap(left_idx + 1, left_idx + 2);
        self.sequence_best_insertion.insert(left_idx + 1, value);
    }

    fn insert_sequence_swap_both(&mut self, value: usize, left_idx: usize) {
        self.sequence_best_insertion.swap(left_idx, left_idx - 1);
        self.sequence_best_insertion
            .swap(left_idx + 1, left_idx + 2);
        self.sequence_best_insertion.insert(left_idx + 1, value);
    }

    fn calculate_path_cost(&self, sequence: &[usize]) -> f64 {
        sequence
            .windows(2)
            .map(|w| self.distance_matrix[w[0]][w[1]])
            .sum()
    }

    fn solve_genetic_algorithm(&mut self) {
        self.generate_chromosomes();
        if self.chromosomes.len() < 2 {
            return;
        }

        for _gen in 0..self.settings.ga.generation {
            if self.chromosomes.len() < 2 {
                break;
            }

            let mut offspring_count = 0;
            let mut min_path_idx = 0;
            let mut tmp_min_path_cost = self.min_path_cost;
            let mut hashed_chromosome_table: HashSet<u64> = HashSet::new();
            let mut tmp_fitness = Vec::new();
            let mut offsprings: Vec<Vec<usize>> = Vec::new();

            let sum_ga_fitness: f64 = self.ga_fitness.iter().sum();
            for _ in 0..self.settings.ga.population {
                let rnd_a: f64 = self.rng.gen::<f64>() * sum_ga_fitness;
                let mut a = 0;
                let mut acc = rnd_a;
                while acc >= 0.0 {
                    acc -= self.ga_fitness[a];
                    a += 1;
                }
                let parent_a = self.chromosomes[a - 1].clone();

                let rnd_b: f64 = self.rng.gen::<f64>() * sum_ga_fitness;
                let mut b = 0;
                let mut acc = rnd_b;
                while acc >= 0.0 {
                    acc -= self.ga_fitness[b];
                    b += 1;
                }
                let parent_b = self.chromosomes[b - 1].clone();

                let slice_1: usize = self.rng.gen_range(1..parent_a.len());
                let slice_2: usize = self.rng.gen_range(1..parent_a.len());
                let (slice_1, slice_2) = if slice_2 < slice_1 {
                    (slice_2, slice_1)
                } else {
                    (slice_1, slice_2)
                };

                let sub_parent_a_set: HashSet<usize> =
                    parent_a[slice_1..slice_2].iter().copied().collect();

                let offset: i32 = self.rng.gen_range(
                    (1 - slice_1 as i32)
                        ..=(parent_a.len().max(parent_b.len()) - slice_2 - 1) as i32,
                );
                let offset = offset as isize;

                let mut offspring = vec![self.source_id];
                let mut idx = 1isize;
                let mut b_idx = 1usize;

                while idx < offset + slice_1 as isize && b_idx < parent_b.len() - 1 {
                    if !sub_parent_a_set.contains(&parent_b[b_idx]) {
                        offspring.push(parent_b[b_idx]);
                        idx += 1;
                    }
                    b_idx += 1;
                }

                if self.rng.gen_bool(0.5) {
                    for i in (offset + slice_1 as isize)..(offset + slice_2 as isize) {
                        offspring.push(parent_a[(i - offset) as usize]);
                    }
                } else {
                    for i in ((offset + slice_1 as isize)..(offset + slice_2 as isize)).rev() {
                        offspring.push(parent_a[(i - offset) as usize]);
                    }
                }

                idx = offset + slice_2 as isize;
                while idx >= offset + slice_2 as isize && b_idx < parent_b.len() - 1 {
                    if !sub_parent_a_set.contains(&parent_b[b_idx]) {
                        offspring.push(parent_b[b_idx]);
                        idx += 1;
                    }
                    b_idx += 1;
                }

                if offspring.last() != Some(&self.target_id) {
                    offspring.push(self.target_id);
                }

                self.apply_shortcut(&mut offspring);

                let hashed = Self::hash_chromosome(&offspring);
                if hashed_chromosome_table.contains(&hashed) {
                    continue;
                }
                hashed_chromosome_table.insert(hashed);

                let path_cost = self.calculate_path_cost(&offspring);
                if path_cost <= self.min_path_cost * 1.1 {
                    tmp_fitness.push(1.0 / path_cost);
                    offsprings.push(offspring);
                    if path_cost < tmp_min_path_cost {
                        tmp_min_path_cost = path_cost;
                        min_path_idx = offspring_count;
                    }
                    offspring_count += 1;
                }
            }

            self.ga_fitness = tmp_fitness;
            self.chromosomes = offsprings;
            if !self.chromosomes.is_empty() {
                self.min_path_cost = tmp_min_path_cost;
                self.sequence_rtsp = self.chromosomes[min_path_idx].clone();
            }
        }
    }

    fn generate_chromosomes(&mut self) {
        let original = self.sequence_rtsp.clone();
        let mut chromosome_count = 0;
        let mut min_path_idx = 0;
        let mut tmp_min_path_cost = self.min_path_cost;
        let mut hashed_chromosome_table: HashSet<u64> = HashSet::new();

        for _ in 0..self.settings.ga.mutation_iter {
            let mut slice = [0usize; 4];
            for s in &mut slice {
                *s = self.rng.gen_range(1..original.len());
            }
            slice.sort_unstable();

            let part1 = original[..slice[0]].to_vec();
            let part2 = original[slice[0]..slice[1]].to_vec();
            let part3 = original[slice[1]..slice[2]].to_vec();
            let part4 = original[slice[2]..slice[3]].to_vec();
            let part5 = original[slice[3]..].to_vec();

            let mut part2 = part2;
            let mut part3 = part3;
            let mut part4 = part4;

            match self.rng.gen_range(0..8) {
                1 => part2.reverse(),
                2 => part3.reverse(),
                3 => part4.reverse(),
                4 => {
                    part2.reverse();
                    part3.reverse();
                }
                5 => {
                    part3.reverse();
                    part4.reverse();
                }
                6 => {
                    part2.reverse();
                    part4.reverse();
                }
                7 => {
                    part2.reverse();
                    part3.reverse();
                    part4.reverse();
                }
                _ => {}
            }

            let mut chromosome = Vec::with_capacity(original.len());
            match self.rng.gen_range(0..5) {
                0 => Self::join_parts(&part1, &part2, &part4, &part3, &part5, &mut chromosome),
                1 => Self::join_parts(&part1, &part3, &part2, &part4, &part5, &mut chromosome),
                2 => Self::join_parts(&part1, &part3, &part4, &part2, &part5, &mut chromosome),
                3 => Self::join_parts(&part1, &part4, &part2, &part3, &part5, &mut chromosome),
                _ => Self::join_parts(&part1, &part4, &part3, &part2, &part5, &mut chromosome),
            }

            self.apply_shortcut(&mut chromosome);

            let hashed = Self::hash_chromosome(&chromosome);
            if hashed_chromosome_table.contains(&hashed) {
                continue;
            }
            hashed_chromosome_table.insert(hashed);

            let path_cost = self.calculate_path_cost(&chromosome);
            if path_cost <= self.min_path_cost * 1.1 {
                self.ga_fitness.push(1.0 / path_cost);
                self.chromosomes.push(chromosome);
                if path_cost < tmp_min_path_cost {
                    tmp_min_path_cost = path_cost;
                    min_path_idx = chromosome_count;
                }
                chromosome_count += 1;
            }
        }

        if !self.chromosomes.is_empty() {
            self.min_path_cost = tmp_min_path_cost;
            self.sequence_rtsp = self.chromosomes[min_path_idx].clone();
        }
    }

    fn join_parts(
        part1: &[usize],
        part2: &[usize],
        part3: &[usize],
        part4: &[usize],
        part5: &[usize],
        chromosome: &mut Vec<usize>,
    ) {
        chromosome.extend_from_slice(part1);
        chromosome.extend_from_slice(part2);
        chromosome.extend_from_slice(part3);
        chromosome.extend_from_slice(part4);
        chromosome.extend_from_slice(part5);
    }

    fn hash_chromosome(chromosome: &[usize]) -> u64 {
        let mut hashed = chromosome.len() as u64;
        for &i in chromosome {
            hashed ^= (i as u64)
                .wrapping_add(0x9e37_79b9)
                .wrapping_add(hashed << 6)
                .wrapping_add(hashed >> 2);
        }
        hashed
    }

    fn refine_sequence(&self, sequence: &mut Vec<usize>) {
        let mut last_idx: HashMap<usize, usize> = HashMap::new();
        for (idx, &item) in sequence.iter().enumerate() {
            last_idx.insert(item, idx);
        }

        let mut visited_node: HashSet<usize> = HashSet::new();
        let mut refined_sequence: Vec<usize> = Vec::new();
        let len = sequence.len();

        let mut i = 0usize;
        while i + 1 < len {
            let item = sequence[i];
            if visited_node.contains(&item) {
                let mut shortcut = false;
                let mut nx = i + 1;
                while nx < len
                    && self.distance_matrix[refined_sequence.last().copied().unwrap_or(item)]
                        [sequence[nx]]
                        .is_infinite()
                    && (visited_node.contains(&sequence[nx]) || nx < last_idx[&sequence[nx]])
                {
                    nx += 1;
                    if nx < len
                        && !visited_node.contains(&sequence[nx])
                        && !self.distance_matrix[refined_sequence.last().copied().unwrap_or(item)]
                            [sequence[nx]]
                            .is_infinite()
                    {
                        refined_sequence.push(sequence[nx]);
                        visited_node.insert(sequence[nx]);
                        shortcut = true;
                        i = nx;
                    }
                }
                if !shortcut && refined_sequence.last() != Some(&item) {
                    refined_sequence.push(item);
                    visited_node.insert(item);
                }
            } else {
                refined_sequence.push(item);
                visited_node.insert(item);
            }
            i += 1;
        }

        if refined_sequence.last() != sequence.last() {
            refined_sequence.push(sequence[len - 1]);
        }
        *sequence = refined_sequence;
    }

    fn apply_shortcut(&self, sequence: &mut Vec<usize>) {
        if self.settings.shortcut != 0 && sequence.len() > self.distance_matrix.len() {
            self.refine_sequence(sequence);
        }
    }
}

impl RtspSolver for EciGenSolver {
    fn solve(
        &mut self,
        distance_matrix: &DistanceMatrix,
        source_id: usize,
        target_id: usize,
    ) -> RtspSolution {
        let (cost, visit_order) = self.solve_rtsp(distance_matrix, source_id, target_id);
        RtspSolution { cost, visit_order }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::GaSettings;

    fn test_settings() -> RtspSettings {
        RtspSettings {
            shortcut: 1,
            swapping: 1,
            genetic: 0,
            ga: GaSettings {
                random_seed: 0,
                mutation_iter: 10,
                population: 10,
                generation: 1,
            },
        }
    }

    #[test]
    fn eci_gen_solves_simple_chain() {
        let matrix = vec![
            vec![0.0, 1.0, 3.0],
            vec![1.0, 0.0, 1.0],
            vec![3.0, 1.0, 0.0],
        ];
        let mut solver = EciGenSolver::new(&test_settings());
        let (cost, seq) = solver.solve_rtsp(&matrix, 0, 2);
        assert_eq!(seq, vec![0, 1, 2]);
        assert!((cost - 2.0).abs() < 1e-9);
    }

    #[test]
    fn eci_gen_dijkstra_only() {
        let matrix = vec![
            vec![0.0, 5.0, 10.0],
            vec![5.0, 0.0, 1.0],
            vec![10.0, 1.0, 0.0],
        ];
        let mut solver = EciGenSolver::new(&test_settings());
        let (cost, seq) = solver.solve_dijkstra(&matrix, 0, 2);
        assert_eq!(seq, vec![0, 1, 2]);
        assert!((cost - 6.0).abs() < 1e-9);
    }

    #[test]
    fn rtsp_trait_exposes_replaceable_solver_contract() {
        let matrix = vec![
            vec![0.0, 1.0, 3.0],
            vec![1.0, 0.0, 1.0],
            vec![3.0, 1.0, 0.0],
        ];
        let mut solver = EciGenSolver::new(&test_settings());
        let solution = RtspSolver::solve(&mut solver, &matrix, 0, 2);
        assert_eq!(solution.visit_order, vec![0, 1, 2]);
        assert!((solution.cost - 2.0).abs() < 1e-9);
    }

    #[test]
    fn shortcut_flag_controls_sequence_refinement() {
        let mut settings = test_settings();
        settings.shortcut = 0;
        let mut disabled = EciGenSolver::new(&settings);
        disabled.distance_matrix = vec![vec![1.0; 5]; 5];
        let original = vec![0, 0, 1, 2, 3, 4, 4];
        let mut sequence = original.clone();
        disabled.apply_shortcut(&mut sequence);
        assert_eq!(sequence, original);

        settings.shortcut = 1;
        let mut enabled = EciGenSolver::new(&settings);
        enabled.distance_matrix = vec![vec![1.0; 5]; 5];
        enabled.apply_shortcut(&mut sequence);
        assert_eq!(sequence, vec![0, 1, 2, 3, 4]);
    }
}
