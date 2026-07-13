mod fake;

pub use fake::FakeMapLoader;

use crate::error::Result;
use crate::graph::AdjacencyGraph;

/// Load a road graph from various sources.
pub trait MapLoader {
    fn load(&self) -> Result<AdjacencyGraph>;
}
