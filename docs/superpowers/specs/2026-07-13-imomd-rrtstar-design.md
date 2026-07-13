# IMOMD-RRTStar Rust 复现设计规格

> 参考论文: [arXiv:2205.14853](https://arxiv.org/abs/2205.14853)  
> 参考实现: [UMich-BipedLab/IMOMD-RRTStar](https://github.com/UMich-BipedLab/IMOMD-RRTStar) (`lib_isrr_release` 分支)

## 1. 目标与范围

在现有 `IMOMD-RRTStar-rs` 仓库中，完整复现 C++ 原版 IMOMD-RRT* 系统，包括：

| 模块 | 原版对应 | 优先级 |
|------|----------|--------|
| 核心 IMOMD-RRT* | `include/imomd_rrt_star/` | P0 |
| ECI-Gen RTSP 求解器 | `eci_gen_tsp_solver.h`, `greedy_tsp.h` | P0 |
| 测试地图 | `fake_map/fake_map.h` | P0 |
| YAML 配置 | `config/algorithm_config.yaml` | P0 |
| OSM 地图解析 | `osm_converter/` | P1 |
| Baseline (Bi-A*, ANA*) | `include/baseline/` | P2 |
| Python 绑定 | maturin + pyo3 | P0 |
| ROS wrapper | `ros_wrapper_isrr_release` | 不在范围 |

**成功标准：**
1. `fake_map_1` / `fake_map_2` 上能找到 source→objectives→target 路径
2. 配置参数与原版 YAML 语义一致
3. `cargo test` 与 `python -m unittest discover test` 全绿
4. 大规模 OSM 地图（Seattle）行为与原版定性一致（允许数值微小差异）

## 2. 架构概览

```
┌─────────────────────────────────────────────────────────┐
│                    Python API (pyo3)                     │
│  ImomdPlanner, RoadGraph, PlanningResult, Config        │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────┐
│              system::PlanningSystem                      │
│  编排 anytime 迭代：RRT* 扩展 ↔ RTSP 求解 ↔ 路径更新    │
└──────┬──────────────────────────────┬───────────────────┘
       │                              │
┌──────▼──────────┐          ┌───────▼────────┐
│ rrt::ImomdRrtStar│          │ rtsp::EciGenSolver│
│ 多树 RRT* 扩展   │          │ 增强 cheapest    │
│ 距离矩阵维护     │          │ insertion + GA   │
└──────┬──────────┘          └──────────────────┘
       │
┌──────▼──────────────────────────────────────────┐
│ graph::RoadGraph (trait)                         │
│  nodes + adjacency list + haversine edge weight   │
└──────┬──────────────────────────────────────────┘
       │
┌──────▼──────────┐   ┌────────────────┐
│ map::FakeMap     │   │ map::OsmLoader  │ (P1)
└──────────────────┘   └────────────────┘
```

## 3. 模块设计

### 3.1 `types` — 核心数据类型

```rust
pub type NodeId = usize;

pub struct Location { pub id: NodeId, pub latitude: f64, pub longitude: f64 }

pub struct Destinations { pub source: NodeId, pub objectives: Vec<NodeId>, pub target: NodeId }

pub struct PlanningResult {
    pub path: Vec<NodeId>,           // 完整节点序列
    pub visit_order: Vec<usize>,     // 目标访问顺序（tree id 序列）
    pub cost: f64,
    pub explored_nodes: usize,
    pub elapsed_secs: f64,
}

pub enum PlannerSystem { Imomd, BiAstar, AnaStar }
```

### 3.2 `graph` — 路网抽象

```rust
pub trait RoadGraph: Send + Sync {
    fn node_count(&self) -> usize;
    fn location(&self, id: NodeId) -> Option<&Location>;
    fn neighbors(&self, id: NodeId) -> impl Iterator<Item = (NodeId, f64)>;
    fn edge_weight(&self, from: NodeId, to: NodeId) -> Option<f64>;
    fn haversine(&self, a: NodeId, b: NodeId) -> Option<f64>;
}

pub struct AdjacencyGraph {
    nodes: Vec<Location>,
    edges: Vec<HashMap<NodeId, f64>>,  // rustc_hash::FxHashMap
}
```

**Rust 优化点：**
- `FxHashMap` 替代 `std::HashMap`（原版 `unordered_map`）
- 邻接表用 `Vec<HashMap<>>` 按 node id 索引，O(1) 查找
- 预计算 haversine 缓存（可选，P1）

### 3.3 `geo` — 地理计算

```rust
pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64;
pub fn bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64;
```

与原版 `computeHaversineDistance` 公式一致（地球半径 6371e3 m）。

### 3.4 `rrt` — IMOMD-RRT* 核心

#### `tree::RrtTree`

对应原版 `tree_t`：
- `parent: FxHashMap<NodeId, NodeId>`
- `children: FxHashMap<NodeId, FxHashSet<NodeId>>`
- `cost: FxHashMap<NodeId, f64>`
- `expandables: FxHashSet<NodeId>`

关键方法：`check_visited`, `update_cost`, `add_node`, `rewire`

#### `planner::ImomdRrtStar`

对应原版 `ImomdRRT` 类，核心循环：

1. `expand_tree_layers()` — 并行扩展所有树层
2. 每棵树 `expand_tree()` 五步：
   - `select_random_vertex()` — goal-bias 采样
   - `steer_new_node()` — 选最近可扩展节点
   - `connect_new_node()` — 最小 cost-to-come 连接
   - `rewire_tree()` — RRT* 重连
   - `update_connection_tree()` — 更新距离矩阵
3. `connect_two_tree()` — 检测树间连接
4. `merge_pseudo_trees()` — pseudo 模式合并
5. `solve_rtsp()` — 触发 ECI-Gen
6. `update_path()` — 根据访问顺序拼接路径

**Rust 优化点：**
- `std::thread` + `crossbeam-channel` 替代 `pthread` + `pthread_cond`
- `rayon` 并行扩展独立树层
- `rand` crate + 可复现种子
- `parking_lot::Mutex` 替代 `pthread_mutex`

**Anytime 接口：**

```rust
pub trait AnytimePlanner {
    fn step(&mut self) -> StepResult;          // 单次迭代
    fn best_solution(&self) -> Option<&PlanningResult>;
    fn is_finished(&self) -> bool;
    fn run_until(&mut self, deadline: Instant) -> PlanningResult;
}
```

### 3.5 `rtsp` — ECI-Gen 求解器

对应原版 `EciGenSolver`：

```rust
pub trait RtspSolver {
    fn solve(&self, distance_matrix: &[Vec<f64>], settings: &RtspSettings)
        -> Vec<usize>;  // tree id 访问顺序
}
```

子模块：
- `greedy.rs` — cheapest insertion 初始解
- `eci_gen.rs` — swapping + genetic algorithm 改进

### 3.6 `config` — 配置

```rust
#[derive(Debug, Deserialize, Serialize)]
pub struct AlgorithmConfig {
    pub general: GeneralConfig,
    pub rrt_params: RrtParams,
    pub destinations: DestinationsConfig,
    pub map: MapConfig,
    pub rtsp_settings: RtspSettings,
}
```

与原版 `algorithm_config.yaml` 字段一一对应。

### 3.7 `map` — 地图加载

```rust
pub trait MapLoader {
    fn load(&self) -> Result<AdjacencyGraph, MapError>;
}

pub struct FakeMapLoader { pub map_type: i32 }  // -1, -2
pub struct OsmMapLoader { pub path: PathBuf, pub filter: OsmFilter }  // P1
```

### 3.8 `baseline` — 对照算法 (P2)

- `BiAstar` — 双向 A*
- `AnaStar` — ANA*

接口与 `ImomdRrtStar` 共享 `AnytimePlanner` trait。

### 3.9 `python` — PyO3 绑定

```python
from IMOMD_RRTStar import ImomdPlanner, FakeMap, AlgorithmConfig

graph = FakeMap.load(-1)
config = AlgorithmConfig.from_yaml("config/algorithm_config.yaml")
planner = ImomdPlanner(graph, config)
result = planner.run_until(seconds=60)
print(result.path, result.cost)
```

暴露类型：
- `ImomdPlanner` — 主入口
- `FakeMap` — 测试地图构造
- `AlgorithmConfig` — YAML 配置
- `PlanningResult` — 结果（path, cost, visit_order）

## 4. 测试策略 (TDD)

### 4.1 单元测试 (`tests/` + `#[cfg(test)]`)

| 测试文件 | 覆盖 |
|----------|------|
| `geo_tests.rs` | haversine 与 C++ 参考值对比 |
| `graph_tests.rs` | fake_map_1/2 节点数、边权重 |
| `tree_tests.rs` | RRT 树增删、cost 传播、rewire |
| `rtsp_tests.rs` | 小矩阵 TSP 已知最优解 |
| `config_tests.rs` | YAML 解析与原版 config 兼容 |

### 4.2 集成测试

| 测试 | 断言 |
|------|------|
| `fake_map_1` source=0, target=2, objectives=[1] | 路径存在，cost > 0 |
| `fake_map_2` bug-trap 场景 | 能 escape trap |
| 确定性种子 | 同种子同结果 |

### 4.3 Python 测试 (`test/`)

- `test_fake_map.py` — Python 加载地图
- `test_planner.py` — 端到端规划
- `test_config.py` — YAML 往返

### 4.4 开发顺序 (TDD 红绿重构)

```
Phase 1: types + geo + graph + fake_map     ← 当前
Phase 2: rrt::tree + tree unit tests
Phase 3: rrt::planner (单目标 RRT* 退化测试)
Phase 4: 多树 + 距离矩阵
Phase 5: rtsp::greedy + rtsp::eci_gen
Phase 6: system 编排 + anytime
Phase 7: config + CLI
Phase 8: OSM loader
Phase 9: baseline
Phase 10: Python 绑定完善
```

## 5. 依赖选型

```toml
[dependencies]
serde = { version = "1", features = ["derive"] }
serde_yaml = "0.9"
rand = "0.8"
rustc-hash = "2"
thiserror = "1"
parking_lot = "0.12"
crossbeam-channel = "0.5"
rayon = "1.10"
pyo3 = { version = "0.18", features = ["extension-module"] }
clap = { version = "4", features = ["derive"] }
log = "0.4"
env_logger = "0.10"

[dev-dependencies]
approx = "0.5"
```

移除不再需要的 `polars-core`（PyArrow demo 解耦到独立 feature）。

## 6. 错误处理

```rust
#[derive(Debug, thiserror::Error)]
pub enum PlannerError {
    #[error("node {0} not found in graph")]
    NodeNotFound(NodeId),
    #[error("graph is disconnected between {0} and {1}")]
    Disconnected(NodeId, NodeId),
    #[error("invalid config: {0}")]
    Config(String),
    #[error("planning timeout after {0}s")]
    Timeout(f64),
}
```

## 7. 与原版差异说明

| 方面 | 原版 C++ | Rust 复现 |
|------|----------|-----------|
| 并发 | pthread + cond/mutex | thread + crossbeam-channel |
| 哈希表 | `std::unordered_map` | `FxHashMap` |
| YAML | ryml | serde_yaml |
| 日志 | 自定义 debugger | log + env_logger |
| CSV 输出 | 自定义 CSVFile | csv crate 或手写 |
| 距离矩阵 | `vector<vector<shared_ptr<...>>>` | `Vec<Vec<Option<f64>>>` |

算法逻辑保持与原版一致，允许浮点累加顺序导致的微小数值差异。

## 8. 文件结构（目标）

```
src/
  lib.rs
  error.rs
  prelude.rs
  types/mod.rs
  geo/mod.rs
  graph/mod.rs
  map/{mod.rs, fake.rs, osm.rs}
  config/mod.rs
  rrt/{mod.rs, tree.rs, planner.rs}
  rtsp/{mod.rs, greedy.rs, eci_gen.rs}
  baseline/{mod.rs, bi_astar.rs, ana_star.rs}
  system/mod.rs
  python/mod.rs
  command/mod.rs
  main.rs
tests/
  geo_tests.rs
  graph_tests.rs
  tree_tests.rs
  config_tests.rs
config/
  algorithm_config.yaml
  osm_way_config.yaml
python/IMOMD_RRTStar/
  __init__.py
  IMOMD_RRTStar.pyi
test/
  test_planner.py
  test_fake_map.py
```
