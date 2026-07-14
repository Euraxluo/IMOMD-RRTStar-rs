use std::collections::{BTreeSet, HashMap};
use std::path::{Path, PathBuf};

use quick_xml::events::Event;
use quick_xml::Reader;
use rustc_hash::FxHashMap;

use crate::config::OsmFilterProperties;
use crate::error::{PlannerError, Result};
use crate::geo::haversine_distance;
use crate::graph::AdjacencyGraph;
use crate::map::MapLoader;
use crate::types::Location;

#[derive(Debug, Clone)]
struct OsmNode {
    latitude: f64,
    longitude: f64,
}

#[derive(Debug, Clone)]
struct OsmWay {
    node_refs: Vec<u64>,
}

/// Load road networks from OpenStreetMap XML files.
/// Maps to C++ `OSMParser`.
pub struct OsmMapLoader {
    pub path: PathBuf,
    pub filter: OsmFilterProperties,
}

impl OsmMapLoader {
    pub fn new(path: impl Into<PathBuf>, filter: OsmFilterProperties) -> Self {
        Self {
            path: path.into(),
            filter,
        }
    }

    pub fn parse_file(path: &Path, filter: &OsmFilterProperties) -> Result<AdjacencyGraph> {
        let xml = std::fs::read_to_string(path)
            .map_err(|e| PlannerError::MapLoad(format!("failed to read OSM file: {e}")))?;
        Self::parse_xml(&xml, filter)
    }

    pub fn parse_xml(xml: &str, filter: &OsmFilterProperties) -> Result<AdjacencyGraph> {
        let mut reader = Reader::from_str(xml);
        reader.config_mut().trim_text(true);

        let mut nodes_map: HashMap<u64, OsmNode> = HashMap::new();
        let mut ways: Vec<OsmWay> = Vec::new();
        // Stable OSM-id ordering keeps internal NodeId values reproducible
        // across processes (std::HashSet iteration is randomly seeded).
        let mut referenced_nodes: BTreeSet<u64> = BTreeSet::new();

        let mut buf = Vec::new();
        let mut current_way_nodes: Vec<u64> = Vec::new();
        let mut current_way_tags: Vec<(String, String)> = Vec::new();
        let mut in_way = false;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(e)) | Ok(Event::Empty(e)) => match e.name().as_ref() {
                    b"node" => {
                        let id = parse_u64_attr(&e, b"id")?;
                        let lat = parse_f64_attr(&e, b"lat")?;
                        let lon = parse_f64_attr(&e, b"lon")?;
                        nodes_map.insert(
                            id,
                            OsmNode {
                                latitude: lat,
                                longitude: lon,
                            },
                        );
                    }
                    b"way" => {
                        in_way = true;
                        current_way_nodes.clear();
                        current_way_tags.clear();
                    }
                    b"nd" if in_way => {
                        current_way_nodes.push(parse_u64_attr(&e, b"ref")?);
                    }
                    b"tag" if in_way => {
                        let key = parse_string_attr(&e, b"k")?;
                        let value = parse_string_attr(&e, b"v")?;
                        current_way_tags.push((key, value));
                    }
                    _ => {}
                },
                Ok(Event::End(e)) if e.name().as_ref() == b"way" => {
                    if way_matches_filter(&current_way_tags, filter) {
                        for &node_ref in &current_way_nodes {
                            referenced_nodes.insert(node_ref);
                        }
                        ways.push(OsmWay {
                            node_refs: current_way_nodes.clone(),
                        });
                    }
                    in_way = false;
                }
                Ok(Event::Eof) => break,
                Ok(_) => {}
                Err(e) => {
                    return Err(PlannerError::MapLoad(format!("XML parse error: {e}")));
                }
            }
            buf.clear();
        }

        let mut id_to_index: HashMap<u64, usize> = HashMap::new();
        let mut nodes: Vec<Location> = Vec::with_capacity(referenced_nodes.len());
        for &osm_id in &referenced_nodes {
            let Some(node) = nodes_map.get(&osm_id) else {
                continue;
            };
            let idx = nodes.len();
            id_to_index.insert(osm_id, idx);
            nodes.push(Location::new(idx, node.latitude, node.longitude));
        }

        let mut edges = vec![FxHashMap::default(); nodes.len()];
        for way in &ways {
            for pair in way.node_refs.windows(2) {
                let Some(&left_idx) = id_to_index.get(&pair[0]) else {
                    continue;
                };
                let Some(&right_idx) = id_to_index.get(&pair[1]) else {
                    continue;
                };
                let weight = haversine_distance(
                    nodes[left_idx].latitude,
                    nodes[left_idx].longitude,
                    nodes[right_idx].latitude,
                    nodes[right_idx].longitude,
                );
                edges[left_idx].insert(right_idx, weight);
                edges[right_idx].insert(left_idx, weight);
            }
        }

        if nodes.is_empty() {
            return Err(PlannerError::MapLoad(
                "OSM parse produced an empty graph".into(),
            ));
        }

        AdjacencyGraph::new(nodes, edges)
    }
}

impl MapLoader for OsmMapLoader {
    fn load(&self) -> Result<AdjacencyGraph> {
        Self::parse_file(&self.path, &self.filter)
    }
}

fn way_matches_filter(tags: &[(String, String)], filter: &OsmFilterProperties) -> bool {
    tags.iter()
        .any(|(key, value)| filter.keys.contains(key) || filter.values.contains(value))
}

fn parse_u64_attr(e: &quick_xml::events::BytesStart<'_>, name: &[u8]) -> Result<u64> {
    let value = attribute_value(e, name)?;
    value.parse().map_err(|_| {
        PlannerError::MapLoad(format!(
            "invalid u64 attribute {:?}",
            std::str::from_utf8(name)
        ))
    })
}

fn parse_f64_attr(e: &quick_xml::events::BytesStart<'_>, name: &[u8]) -> Result<f64> {
    let value = attribute_value(e, name)?;
    value.parse().map_err(|_| {
        PlannerError::MapLoad(format!(
            "invalid f64 attribute {:?}",
            std::str::from_utf8(name)
        ))
    })
}

fn parse_string_attr(e: &quick_xml::events::BytesStart<'_>, name: &[u8]) -> Result<String> {
    attribute_value(e, name)
}

fn attribute_value(e: &quick_xml::events::BytesStart<'_>, name: &[u8]) -> Result<String> {
    e.attributes()
        .find(|a| {
            a.as_ref()
                .map(|attr| attr.key.as_ref() == name)
                .unwrap_or(false)
        })
        .ok_or_else(|| {
            PlannerError::MapLoad(format!("missing attribute {:?}", std::str::from_utf8(name)))
        })?
        .map_err(|e| PlannerError::MapLoad(e.to_string()))
        .and_then(|attr| {
            String::from_utf8(attr.value.into_owned())
                .map_err(|e| PlannerError::MapLoad(e.to_string()))
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::OsmWayConfig;
    use crate::graph::RoadGraph;

    #[test]
    fn parse_tiny_osm_fixture() {
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/tiny.osm");
        let filter = OsmWayConfig::from_yaml_file(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("config/osm_way_config.yaml"),
        )
        .unwrap()
        .filter_properties()
        .unwrap();

        let graph = OsmMapLoader::parse_file(&fixture, &filter).unwrap();
        assert_eq!(graph.node_count(), 3);
        let mut degrees: Vec<usize> = (0..graph.node_count())
            .map(|i| graph.neighbors(i).len())
            .collect();
        degrees.sort_unstable();
        assert_eq!(degrees, vec![1, 1, 2]);
    }

    #[test]
    fn repeated_osm_loads_keep_node_ids_stable() {
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/tiny.osm");
        let filter = OsmWayConfig::from_yaml_file(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("config/osm_way_config.yaml"),
        )
        .unwrap()
        .filter_properties()
        .unwrap();

        let first = OsmMapLoader::parse_file(&fixture, &filter).unwrap();
        let second = OsmMapLoader::parse_file(&fixture, &filter).unwrap();
        assert_eq!(first.nodes(), second.nodes());
        assert_eq!(first.edges(), second.edges());
    }
}
