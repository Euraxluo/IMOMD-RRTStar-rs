mod custom;
mod fake;
mod osm;

pub use custom::CustomGraphLoader;
pub use fake::FakeMapLoader;
pub use osm::OsmMapLoader;

use crate::error::Result;
use crate::graph::AdjacencyGraph;

/// Load a road graph from various sources.
pub trait MapLoader {
    fn load(&self) -> Result<AdjacencyGraph>;
}
