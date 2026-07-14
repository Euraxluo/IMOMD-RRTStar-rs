use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rustc_hash::FxHashMap;
use std::path::PathBuf;
use std::sync::Arc;

use crate::config::{AlgorithmConfig, OsmWayConfig};
use crate::graph::{AdjacencyGraph, RoadGraph, TrafficGraph, TrafficLevel};
use crate::map::{CustomGraphLoader, FakeMapLoader, MapLoader, OsmMapLoader};
use crate::rrt::ImomdRrtStar;
use crate::types::{Destinations, GraphUpdateStats, Location, PlanningResult, StepStatus};

#[pyclass(name = "AlgorithmConfig", frozen)]
#[derive(Clone)]
pub struct PyAlgorithmConfig {
    inner: AlgorithmConfig,
}

#[pymethods]
impl PyAlgorithmConfig {
    #[staticmethod]
    fn from_yaml(path: &str) -> PyResult<Self> {
        let inner = AlgorithmConfig::from_yaml_file(PathBuf::from(path).as_path())
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self { inner })
    }

    #[staticmethod]
    fn from_yaml_string(yaml: &str) -> PyResult<Self> {
        let inner = AlgorithmConfig::from_yaml_str(yaml)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self { inner })
    }

    fn to_yaml_string(&self) -> PyResult<String> {
        self.inner
            .to_yaml_string()
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    #[getter]
    fn source(&self) -> usize {
        self.inner.destinations.source_id
    }

    #[getter]
    fn objectives(&self) -> Vec<usize> {
        self.inner.destinations.objective_ids.clone()
    }

    #[getter]
    fn target(&self) -> usize {
        self.inner.destinations.target_id
    }

    #[getter]
    fn max_iter(&self) -> usize {
        self.inner.general.max_iter
    }

    #[getter]
    fn max_time_secs(&self) -> u64 {
        self.inner.general.max_time
    }

    #[getter]
    fn goal_bias(&self) -> f64 {
        self.inner.rrt_params.goal_bias
    }
}

#[pyclass(name = "GraphUpdateStats")]
#[derive(Clone)]
pub struct PyGraphUpdateStats {
    #[pyo3(get)]
    pub previous_tree_nodes: usize,
    #[pyo3(get)]
    pub retained_tree_nodes: usize,
    #[pyo3(get)]
    pub pruned_tree_nodes: usize,
}

impl From<GraphUpdateStats> for PyGraphUpdateStats {
    fn from(stats: GraphUpdateStats) -> Self {
        Self {
            previous_tree_nodes: stats.previous_tree_nodes,
            retained_tree_nodes: stats.retained_tree_nodes,
            pruned_tree_nodes: stats.pruned_tree_nodes,
        }
    }
}

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

#[pyclass(name = "CustomGraph")]
pub struct PyCustomGraph;

#[pymethods]
impl PyCustomGraph {
    #[staticmethod]
    fn load(path: &str) -> PyResult<PyAdjacencyGraph> {
        let graph = CustomGraphLoader::new(path)
            .load()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PyAdjacencyGraph {
            inner: Arc::new(graph),
        })
    }
}

#[pyclass(name = "OsmMap")]
pub struct PyOsmMap;

#[pymethods]
impl PyOsmMap {
    #[staticmethod]
    #[pyo3(signature = (osm_path, *, osm_way_config="config/osm_way_config.yaml"))]
    fn load(osm_path: &str, osm_way_config: &str) -> PyResult<PyAdjacencyGraph> {
        let cfg = OsmWayConfig::from_yaml_file(PathBuf::from(osm_way_config).as_path())
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let filter = cfg
            .filter_properties()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let graph = OsmMapLoader::new(osm_path, filter)
            .load()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PyAdjacencyGraph {
            inner: Arc::new(graph),
        })
    }
}

#[pyclass(name = "TrafficGraph")]
pub struct PyTrafficGraph {
    inner: TrafficGraph,
}

#[pymethods]
impl PyTrafficGraph {
    #[staticmethod]
    fn from_edges(nodes: Vec<(f64, f64)>, edges: Vec<(usize, usize, f64)>) -> PyResult<Self> {
        if nodes.is_empty() {
            return Err(PyValueError::new_err("nodes must not be empty"));
        }
        let locations: Vec<Location> = nodes
            .into_iter()
            .enumerate()
            .map(|(id, (lat, lon))| Location::new(id, lat, lon))
            .collect();
        let mut adjacency = vec![FxHashMap::default(); locations.len()];
        for (from, to, weight) in edges {
            if from >= locations.len() {
                return Err(PyValueError::new_err(format!("node {from} not found")));
            }
            if to >= locations.len() {
                return Err(PyValueError::new_err(format!("node {to} not found")));
            }
            if from == to {
                return Err(PyValueError::new_err("self edges are not supported"));
            }
            if !weight.is_finite() || weight <= 0.0 {
                return Err(PyValueError::new_err(format!(
                    "edge ({from}, {to}) has invalid weight {weight}"
                )));
            }
            adjacency[from].insert(to, weight);
            adjacency[to].insert(from, weight);
        }
        let base = AdjacencyGraph::new(locations, adjacency)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            inner: TrafficGraph::new(base),
        })
    }

    #[staticmethod]
    fn load_fake(map_type: i32) -> PyResult<Self> {
        let base = FakeMapLoader::new(map_type)
            .load()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            inner: TrafficGraph::new(base),
        })
    }

    #[staticmethod]
    #[pyo3(signature = (osm_path, *, osm_way_config="config/osm_way_config.yaml"))]
    fn load_osm(osm_path: &str, osm_way_config: &str) -> PyResult<Self> {
        let cfg = OsmWayConfig::from_yaml_file(PathBuf::from(osm_way_config).as_path())
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let filter = cfg
            .filter_properties()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let base = OsmMapLoader::new(osm_path, filter)
            .load()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            inner: TrafficGraph::new(base),
        })
    }

    #[getter]
    fn node_count(&self) -> usize {
        self.inner.node_count()
    }

    fn set_edge_traffic(&mut self, from: usize, to: usize, level: &str) -> PyResult<()> {
        if self.inner.base().location(from).is_none() {
            return Err(PyValueError::new_err(format!("node {from} not found")));
        }
        if self.inner.base().location(to).is_none() {
            return Err(PyValueError::new_err(format!("node {to} not found")));
        }
        if self.inner.base().edge_weight(from, to).is_none()
            && self.inner.base().edge_weight(to, from).is_none()
        {
            return Err(PyValueError::new_err(format!(
                "nodes {from} and {to} are not connected by an edge"
            )));
        }
        let level = TrafficLevel::from_label(level)
            .ok_or_else(|| PyValueError::new_err(format!("unknown traffic level: {level}")))?;
        self.inner.set_edge_level(from, to, level);
        Ok(())
    }

    fn set_zone_traffic(&mut self, nodes: Vec<usize>, level: &str) -> PyResult<()> {
        if let Some(node) = nodes
            .iter()
            .copied()
            .find(|&node| self.inner.base().location(node).is_none())
        {
            return Err(PyValueError::new_err(format!("node {node} not found")));
        }
        let level = TrafficLevel::from_label(level)
            .ok_or_else(|| PyValueError::new_err(format!("unknown traffic level: {level}")))?;
        self.inner.set_zone_level(&nodes, level);
        Ok(())
    }

    fn clear_traffic(&mut self) {
        self.inner.clear_traffic();
    }

    fn materialize(&self) -> PyResult<PyAdjacencyGraph> {
        let graph = self
            .inner
            .materialize()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PyAdjacencyGraph {
            inner: Arc::new(graph),
        })
    }

    fn export_view(&self) -> PyResult<PyObject> {
        Python::with_gil(|py| {
            let view = self.inner.export_view();
            let nodes = pyo3::types::PyList::empty(py);
            for n in view.nodes {
                let dict = pyo3::types::PyDict::new(py);
                dict.set_item("id", n.id)?;
                dict.set_item("lat", n.latitude)?;
                dict.set_item("lon", n.longitude)?;
                nodes.append(dict)?;
            }
            let edges = pyo3::types::PyList::empty(py);
            for e in view.edges {
                let dict = pyo3::types::PyDict::new(py);
                dict.set_item("from", e.from)?;
                dict.set_item("to", e.to)?;
                dict.set_item("base_weight", e.base_weight)?;
                dict.set_item("weight", e.effective_weight)?;
                dict.set_item("level", e.level)?;
                edges.append(dict)?;
            }
            let out = pyo3::types::PyDict::new(py);
            out.set_item("nodes", nodes)?;
            out.set_item("edges", edges)?;
            Ok(out.into())
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
    #[pyo3(signature = (graph, source_or_config, objectives=None, target=None, *, max_iter=1_000_000, max_time_secs=60, goal_bias=1.0))]
    fn new(
        graph: &PyAdjacencyGraph,
        source_or_config: &Bound<'_, PyAny>,
        objectives: Option<Vec<usize>>,
        target: Option<usize>,
        max_iter: usize,
        max_time_secs: u64,
        goal_bias: f64,
    ) -> PyResult<Self> {
        let (config, destinations) = if let Ok(py_config) =
            source_or_config.extract::<PyRef<'_, PyAlgorithmConfig>>()
        {
            if objectives.is_some() || target.is_some() {
                return Err(PyValueError::new_err(
                    "objectives/target must be omitted when using AlgorithmConfig",
                ));
            }
            if py_config.inner.general.system != 0 {
                return Err(PyValueError::new_err(
                    "ImomdPlanner requires config.general.system = 0",
                ));
            }
            let config = py_config.inner.clone();
            let destinations = Destinations {
                source: config.destinations.source_id,
                objectives: config.destinations.objective_ids.clone(),
                target: config.destinations.target_id,
            };
            (config, destinations)
        } else {
            let source = source_or_config
                .extract::<usize>()
                .map_err(|_| PyValueError::new_err("expected source node id or AlgorithmConfig"))?;
            let objectives = objectives
                .ok_or_else(|| PyValueError::new_err("objectives are required with a source id"))?;
            let target = target
                .ok_or_else(|| PyValueError::new_err("target is required with a source id"))?;
            let config = AlgorithmConfig::from_yaml_str(&format!(
                r#"
general: {{ system: 0, pseudo: 0, log_data: 0, print_path: 0, max_iter: {max_iter}, max_time: {max_time_secs} }}
rrt_params: {{ goal_bias: {goal_bias}, random_seed: 0 }}
destinations: {{ source_id: {source}, objective_ids: {objectives:?}, target_id: {target} }}
map: {{ type: -1, path: "", name: "" }}
rtsp_settings: {{ shortcut: 1, swapping: 1, genetic: 1, ga: {{ random_seed: 0, mutation_iter: 10000, population: 1000, generation: 5 }} }}
"#
            ))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
            let destinations = Destinations {
                source,
                objectives,
                target,
            };
            (config, destinations)
        };

        let planner = ImomdRrtStar::new(Arc::clone(&graph.inner), destinations, config)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;

        Ok(Self { planner })
    }

    fn run_for(&mut self, py: Python<'_>, seconds: f64) -> PyResult<PyPlanningResult> {
        if !seconds.is_finite() || seconds <= 0.0 {
            return Err(PyValueError::new_err("seconds must be positive and finite"));
        }
        let duration = std::time::Duration::from_secs_f64(seconds);
        match py.allow_threads(|| self.planner.run_for(duration)) {
            Ok(result) => Ok(PyPlanningResult::from(result)),
            Err(e) => Err(PyValueError::new_err(e.to_string())),
        }
    }

    fn run_until(&mut self, py: Python<'_>, seconds: f64) -> PyResult<PyPlanningResult> {
        self.run_for(py, seconds)
    }

    fn update_graph(
        &mut self,
        py: Python<'_>,
        graph: &PyAdjacencyGraph,
    ) -> PyResult<PyGraphUpdateStats> {
        let graph = Arc::clone(&graph.inner);
        py.allow_threads(|| self.planner.update_graph(graph))
            .map(PyGraphUpdateStats::from)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    fn step(&mut self, py: Python<'_>) -> PyResult<PyObject> {
        let step = py
            .allow_threads(|| self.planner.step())
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let dict = pyo3::types::PyDict::new(py);
        dict.set_item(
            "status",
            match step.status {
                StepStatus::Expanded => "expanded",
                StepStatus::Connected => "connected",
                StepStatus::PathImproved => "path_improved",
                StepStatus::Finished => "finished",
            },
        )?;
        dict.set_item("iteration", step.iteration)?;
        dict.set_item("best_cost", step.best_cost)?;
        Ok(dict.into())
    }

    fn best_result(&self) -> PyResult<Option<PyPlanningResult>> {
        Ok(self
            .planner
            .best_solution()
            .map(|r| PyPlanningResult::from(r.clone())))
    }

    fn tree_count(&self) -> usize {
        self.planner.init_trees().len()
    }

    #[getter]
    fn is_finished(&self) -> bool {
        self.planner.is_finished()
    }
}

#[pyclass(name = "NavigationSession")]
pub struct PyNavigationSession {
    session: crate::navigation::NavigationSession,
}

fn plan_update_to_dict(py: Python<'_>, update: &crate::navigation::PlanUpdate) -> PyResult<PyObject> {
    let dict = pyo3::types::PyDict::new(py);
    dict.set_item("sequence", update.sequence)?;
    dict.set_item(
        "reason",
        match update.reason {
            crate::navigation::UpdateReason::Expanded => "expanded",
            crate::navigation::UpdateReason::Connected => "connected",
            crate::navigation::UpdateReason::Improved => "improved",
            crate::navigation::UpdateReason::Finished => "finished",
            crate::navigation::UpdateReason::EgoReseed => "ego_reseed",
            crate::navigation::UpdateReason::TrafficWarmStart => "traffic_warm_start",
            crate::navigation::UpdateReason::Fresh => "fresh",
            crate::navigation::UpdateReason::Resume => "resume",
            crate::navigation::UpdateReason::GreedyInit => "greedy_init",
            crate::navigation::UpdateReason::ExactOptimal => "exact_optimal",
        },
    )?;
    dict.set_item("path", update.path.clone())?;
    dict.set_item("cost", update.cost)?;
    dict.set_item("visit_order", update.visit_order.clone())?;
    dict.set_item("explored_nodes", update.explored_nodes)?;
    dict.set_item("replan_mode", &update.replan_mode)?;
    dict.set_item("ego_node", update.ego_node)?;
    dict.set_item("algorithm_id", &update.algorithm_id)?;
    if let Some(stats) = update.tree_update {
        let stats_dict = pyo3::types::PyDict::new(py);
        stats_dict.set_item("previous_tree_nodes", stats.previous_tree_nodes)?;
        stats_dict.set_item("retained_tree_nodes", stats.retained_tree_nodes)?;
        stats_dict.set_item("pruned_tree_nodes", stats.pruned_tree_nodes)?;
        dict.set_item("tree_update", stats_dict)?;
    } else {
        dict.set_item("tree_update", py.None())?;
    }
    Ok(dict.into())
}

#[pymethods]
impl PyNavigationSession {
    #[new]
    #[pyo3(signature = (algorithm="imomd"))]
    fn new(algorithm: &str) -> PyResult<Self> {
        let plugin: Box<dyn crate::navigation::PlannerPlugin> = match algorithm {
            "imomd" => Box::new(crate::navigation::ImomdPlugin::with_default_config()),
            other => {
                return Err(PyValueError::new_err(format!(
                    "unsupported navigation algorithm '{other}' (available: imomd)"
                )));
            }
        };
        Ok(Self {
            session: crate::navigation::NavigationSession::new(plugin),
        })
    }

    #[getter]
    fn algorithm_id(&self) -> &str {
        self.session.algorithm_id()
    }

    #[getter]
    fn ego_node(&self) -> Option<usize> {
        self.session.ego_node()
    }

    fn set_graph(&mut self, graph: &PyAdjacencyGraph) {
        self.session.set_graph(Arc::clone(&graph.inner));
    }

    fn snap_ego(&self, latitude: f64, longitude: f64) -> PyResult<usize> {
        self.session
            .snap_ego(latitude, longitude)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    #[pyo3(signature = (source, objectives, target, budget_secs=0.5))]
    fn set_destinations(
        &mut self,
        py: Python<'_>,
        source: usize,
        objectives: Vec<usize>,
        target: usize,
        budget_secs: f64,
    ) -> PyResult<Vec<PyObject>> {
        let budget = std::time::Duration::from_secs_f64(budget_secs.max(0.0));
        let updates = py
            .allow_threads(|| {
                self.session.handle(
                    crate::navigation::DomainEvent::DestinationsSet {
                        source,
                        objectives,
                        target,
                    },
                    budget,
                )
            })
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        updates
            .iter()
            .map(|u| plan_update_to_dict(py, u))
            .collect()
    }

    #[pyo3(signature = (budget_secs=0.5))]
    fn on_traffic_changed(&mut self, py: Python<'_>, budget_secs: f64) -> PyResult<Vec<PyObject>> {
        let budget = std::time::Duration::from_secs_f64(budget_secs.max(0.0));
        let updates = py
            .allow_threads(|| {
                self.session
                    .handle(crate::navigation::DomainEvent::TrafficChanged, budget)
            })
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        updates
            .iter()
            .map(|u| plan_update_to_dict(py, u))
            .collect()
    }

    #[pyo3(signature = (ego_node, budget_secs=0.5))]
    fn on_ego_moved(
        &mut self,
        py: Python<'_>,
        ego_node: usize,
        budget_secs: f64,
    ) -> PyResult<Vec<PyObject>> {
        let budget = std::time::Duration::from_secs_f64(budget_secs.max(0.0));
        let updates = py
            .allow_threads(|| {
                self.session.handle(
                    crate::navigation::DomainEvent::EgoMoved { ego_node },
                    budget,
                )
            })
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        updates
            .iter()
            .map(|u| plan_update_to_dict(py, u))
            .collect()
    }

    #[pyo3(signature = (budget_secs=0.3))]
    fn continue_search(&mut self, py: Python<'_>, budget_secs: f64) -> PyResult<Vec<PyObject>> {
        let budget = std::time::Duration::from_secs_f64(budget_secs.max(0.0));
        let updates = py
            .allow_threads(|| {
                self.session
                    .handle(crate::navigation::DomainEvent::ContinueSearch, budget)
            })
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        updates
            .iter()
            .map(|u| plan_update_to_dict(py, u))
            .collect()
    }

    fn best(&self) -> Option<PyPlanningResult> {
        self.session.best().cloned().map(Into::into)
    }
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyAlgorithmConfig>()?;
    m.add_class::<PyGraphUpdateStats>()?;
    m.add_class::<PyTrafficGraph>()?;
    m.add_class::<PyFakeMap>()?;
    m.add_class::<PyCustomGraph>()?;
    m.add_class::<PyOsmMap>()?;
    m.add_class::<PyAdjacencyGraph>()?;
    m.add_class::<PyImomdPlanner>()?;
    m.add_class::<PyPlanningResult>()?;
    m.add_class::<PyNavigationSession>()?;
    Ok(())
}
