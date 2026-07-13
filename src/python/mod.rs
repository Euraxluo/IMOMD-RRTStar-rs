use pyo3::prelude::*;
use pyo3::exceptions::PyNotImplementedError;
use std::sync::Arc;

use crate::config::AlgorithmConfig;
use crate::graph::{AdjacencyGraph, RoadGraph};
use crate::map::{FakeMapLoader, MapLoader};
use crate::rrt::ImomdRrtStar;
use crate::types::{Destinations, PlanningResult};

#[pyclass(name = "PlanningResult")]
#[derive(Clone)]
pub struct PyPlanningResult {
    #[pyo3(get)]
    pub path: Vec<usize>,
    #[pyo3(get)]
    pub visit_order: Vec<usize>,
    #[pyo3(get)]
    pub cost: f64,
    #[pyo3(get)]
    pub explored_nodes: usize,
    #[pyo3(get)]
    pub elapsed_secs: f64,
}

impl From<PlanningResult> for PyPlanningResult {
    fn from(r: PlanningResult) -> Self {
        Self {
            path: r.path,
            visit_order: r.visit_order,
            cost: r.cost,
            explored_nodes: r.explored_nodes,
            elapsed_secs: r.elapsed_secs,
        }
    }
}

#[pyclass(name = "FakeMap")]
pub struct PyFakeMap;

#[pymethods]
impl PyFakeMap {
    #[staticmethod]
    fn load(map_type: i32) -> PyResult<PyAdjacencyGraph> {
        let graph = FakeMapLoader::new(map_type)
            .load()
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        Ok(PyAdjacencyGraph {
            inner: Arc::new(graph),
        })
    }
}

#[pyclass(name = "AdjacencyGraph")]
pub struct PyAdjacencyGraph {
    inner: Arc<AdjacencyGraph>,
}

#[pymethods]
impl PyAdjacencyGraph {
    #[getter]
    fn node_count(&self) -> usize {
        self.inner.node_count()
    }
}

#[pyclass(name = "ImomdPlanner")]
pub struct PyImomdPlanner {
    planner: ImomdRrtStar,
}

#[pymethods]
impl PyImomdPlanner {
    #[new]
    fn new(
        graph: &PyAdjacencyGraph,
        source: usize,
        objectives: Vec<usize>,
        target: usize,
        max_iter: usize,
        max_time_secs: u64,
        goal_bias: f64,
    ) -> PyResult<Self> {
        let config = AlgorithmConfig::from_yaml_str(&format!(
            r#"
general: {{ system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: {max_iter}, max_time: {max_time_secs} }}
rrt_params: {{ goal_bias: {goal_bias}, random_seed: 0 }}
destinations: {{ source_id: {source}, objective_ids: {objectives:?}, target_id: {target} }}
map: {{ type: -1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 1, ga: {{ random_seed: 0, mutation_iter: 10000, population: 1000, generation: 5 }} }}
"#
        ))
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;

        let destinations = Destinations {
            source,
            objectives,
            target,
        };

        let planner = ImomdRrtStar::new(Arc::clone(&graph.inner), destinations, config)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;

        Ok(Self { planner })
    }

    fn run_for(&mut self, seconds: f64) -> PyResult<PyPlanningResult> {
        let duration = std::time::Duration::from_secs_f64(seconds);
        match self.planner.run_for(duration) {
            Ok(result) => Ok(PyPlanningResult::from(result)),
            Err(e) => Err(PyNotImplementedError::new_err(e.to_string())),
        }
    }

    fn tree_count(&self) -> usize {
        self.planner.init_trees().len()
    }
}

pub fn register(m: &PyModule) -> PyResult<()> {
    m.add_class::<PyFakeMap>()?;
    m.add_class::<PyAdjacencyGraph>()?;
    m.add_class::<PyImomdPlanner>()?;
    m.add_class::<PyPlanningResult>()?;
    Ok(())
}
