use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::error::{PlannerError, Result};
use crate::geo::haversine_distance;
use crate::graph::AdjacencyGraph;
use crate::map::MapLoader;
use crate::types::Location;
use rustc_hash::FxHashMap;

#[derive(Debug, Clone, Deserialize)]
struct CustomNodeSpec {
    #[serde(default)]
    id: Option<usize>,
    latitude: f64,
    longitude: f64,
}

#[derive(Debug, Clone, Deserialize)]
struct CustomGraphFile {
    nodes: Vec<CustomNodeSpec>,
    edges: Vec<[usize; 2]>,
}

/// Load a user-defined road graph from YAML (`map.type == 0`).
///
/// Format:
/// ```yaml
/// nodes:
///   - { latitude: 0.0, longitude: 0.0 }
///   - { latitude: 1.0, longitude: 0.0 }
/// edges:
///   - [0, 1]
/// ```
pub struct CustomGraphLoader {
    pub path: PathBuf,
}

impl CustomGraphLoader {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        Self { path: path.into() }
    }

    pub fn parse_file(path: &Path) -> Result<AdjacencyGraph> {
        let contents = std::fs::read_to_string(path)
            .map_err(|e| PlannerError::MapLoad(format!("failed to read graph file: {e}")))?;
        Self::parse_yaml(&contents)
    }

    pub fn parse_yaml(yaml: &str) -> Result<AdjacencyGraph> {
        let spec: CustomGraphFile = serde_yaml::from_str(yaml)
            .map_err(|e| PlannerError::MapLoad(format!("invalid custom graph yaml: {e}")))?;

        if spec.nodes.is_empty() {
            return Err(PlannerError::MapLoad(
                "custom graph must contain at least one node".into(),
            ));
        }

        let nodes: Vec<Location> = spec
            .nodes
            .iter()
            .enumerate()
            .map(|(idx, node)| {
                if let Some(id) = node.id {
                    if id != idx {
                        return Err(PlannerError::MapLoad(format!(
                            "node id {id} does not match index {idx}"
                        )));
                    }
                }
                Ok(Location::new(idx, node.latitude, node.longitude))
            })
            .collect::<Result<_>>()?;

        let mut edges = vec![FxHashMap::default(); nodes.len()];
        for [left, right] in spec.edges {
            if left >= nodes.len() || right >= nodes.len() {
                return Err(PlannerError::MapLoad(format!(
                    "edge ({left}, {right}) references unknown node"
                )));
            }
            if left == right {
                continue;
            }
            let weight = haversine_distance(
                nodes[left].latitude,
                nodes[left].longitude,
                nodes[right].latitude,
                nodes[right].longitude,
            );
            edges[left].insert(right, weight);
            edges[right].insert(left, weight);
        }

        AdjacencyGraph::new(nodes, edges)
    }
}

impl MapLoader for CustomGraphLoader {
    fn load(&self) -> Result<AdjacencyGraph> {
        Self::parse_file(&self.path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::RoadGraph;

    #[test]
    fn parse_custom_graph_fixture_matches_fake_map_1_topology() {
        let fixture =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/custom_graph.yaml");
        let graph = CustomGraphLoader::parse_file(&fixture).unwrap();
        assert_eq!(graph.node_count(), 4);
        assert_eq!(graph.neighbors(0).len(), 2);
        assert_eq!(graph.neighbors(2).len(), 1);
    }

    #[test]
    fn rejects_mismatched_node_id() {
        let yaml = r#"
nodes:
  - { id: 5, latitude: 0.0, longitude: 0.0 }
edges: []
"#;
        let err = CustomGraphLoader::parse_yaml(yaml).unwrap_err();
        assert!(err.to_string().contains("does not match index"));
    }
}
