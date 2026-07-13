use crate::error::{PlannerError, Result};
use crate::geo::haversine_distance;
use crate::graph::AdjacencyGraph;
use crate::map::MapLoader;
use crate::types::Location;
use rustc_hash::FxHashMap;

/// Test maps from C++ `fake_map.h` (map_type -1 and -2).
pub struct FakeMapLoader {
    pub map_type: i32,
}

impl FakeMapLoader {
    pub fn new(map_type: i32) -> Self {
        Self { map_type }
    }
}

impl MapLoader for FakeMapLoader {
    fn load(&self) -> Result<AdjacencyGraph> {
        match self.map_type {
            -1 => load_map_1(),
            -2 => load_map_2(),
            other => Err(PlannerError::MapLoad(format!(
                "invalid fake map type: {other}"
            ))),
        }
    }
}

fn edge_weight(a: &Location, b: &Location) -> f64 {
    haversine_distance(a.latitude, a.longitude, b.latitude, b.longitude)
}

fn load_map_1() -> Result<AdjacencyGraph> {
    //  0 --- 1 --- 2
    //   \   /
    //     3
    let nodes = vec![
        Location::new(0, 0.0, 0.0),
        Location::new(1, 2.0, 0.0),
        Location::new(2, 4.0, 0.0),
        Location::new(3, 1.0, -1.0),
    ];

    let mut g0 = FxHashMap::default();
    g0.insert(1, edge_weight(&nodes[0], &nodes[1]));
    g0.insert(3, edge_weight(&nodes[0], &nodes[3]));

    let mut g1 = FxHashMap::default();
    g1.insert(0, edge_weight(&nodes[1], &nodes[0]));
    g1.insert(2, edge_weight(&nodes[1], &nodes[2]));
    g1.insert(3, edge_weight(&nodes[1], &nodes[3]));

    let mut g2 = FxHashMap::default();
    g2.insert(1, edge_weight(&nodes[2], &nodes[1]));

    let mut g3 = FxHashMap::default();
    g3.insert(0, edge_weight(&nodes[3], &nodes[0]));
    g3.insert(1, edge_weight(&nodes[3], &nodes[1]));

    AdjacencyGraph::new(nodes, vec![g0, g1, g2, g3])
}

fn load_map_2() -> Result<AdjacencyGraph> {
    let nodes = vec![
        Location::new(0, -0.1, 2.1),
        Location::new(1, 0.1, 2.1),
        Location::new(2, 0.0, 2.0),
        Location::new(3, -1.0, 0.0),
        Location::new(4, 1.0, 0.0),
        Location::new(5, 0.0, -1.0),
        Location::new(6, 0.0, -1.1),
    ];

    let edges: Vec<Vec<(usize, usize)>> = vec![
        vec![(0, 1), (0, 2)],
        vec![(1, 0), (1, 2)],
        vec![(2, 0), (2, 1), (2, 3), (2, 4)],
        vec![(3, 2), (3, 5)],
        vec![(4, 2), (4, 5)],
        vec![(5, 3), (5, 4), (5, 6)],
        vec![(6, 5)],
    ];

    let mut adj: Vec<FxHashMap<usize, f64>> = vec![FxHashMap::default(); nodes.len()];
    for (from, pairs) in edges.into_iter().enumerate() {
        for (to, _) in pairs {
            adj[from].insert(to, edge_weight(&nodes[from], &nodes[to]));
        }
    }

    AdjacencyGraph::new(nodes, adj)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::RoadGraph;

    #[test]
    fn fake_map_1_has_four_nodes() {
        let g = FakeMapLoader::new(-1).load().unwrap();
        assert_eq!(g.node_count(), 4);
    }

    #[test]
    fn fake_map_2_has_seven_nodes() {
        let g = FakeMapLoader::new(-2).load().unwrap();
        assert_eq!(g.node_count(), 7);
    }

    #[test]
    fn fake_map_1_node0_has_two_neighbors() {
        let g = FakeMapLoader::new(-1).load().unwrap();
        assert_eq!(g.neighbors(0).len(), 2);
    }
}
