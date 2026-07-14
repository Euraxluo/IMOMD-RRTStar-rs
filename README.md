# IMOMD-RRTStar-rs

Rust reproduction of **Informable Multi-Objective and Multi-Directional RRT\***
([paper](https://arxiv.org/abs/2205.14853),
[original C++](https://github.com/UMich-BipedLab/IMOMD-RRTStar/tree/lib_isrr_release)).
The implementation keeps the paper/C++ algorithm structure while exposing a
pause/resume Rust API, typed Python package, OSM loaders, and a FastAPI V2X
demonstration.

## Implemented scope

| Area | Status |
|---|---|
| Multi-tree IMOMD-RRT*, rewiring, pseudo-objective mode | Implemented |
| ECI-Gen (cheapest insertion, swapping, shortcut, genetic refinement) | Implemented |
| Bi-A* and ANA* baselines | Implemented |
| Fake, custom YAML, and OSM maps | Implemented |
| Original YAML configuration and CLI | Implemented |
| Pause/resume anytime API and dynamic graph warm-start | Implemented |
| PyO3/maturin package with PEP 561 types | Implemented |
| FastAPI + Canvas V2X simulation | Implemented |
| ROS wrapper | Out of scope |

## Quick start

```bash
cargo test --all-targets
cargo run -- --config config/algorithm_config.yaml
```

The CLI prints elapsed time, route cost, explored tree size, and—when
`general.print_path: 1`—the complete node path.

### Python

Supported Python versions are 3.8–3.13.

```bash
python -m venv .venv
.venv/bin/pip install maturin
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  .venv/bin/maturin develop --features python,extension-module
.venv/bin/python -m unittest discover test -v
```

```python
from IMOMD_RRTStar import AlgorithmConfig, FakeMap, ImomdPlanner

graph = FakeMap.load(-1)
config = AlgorithmConfig.from_yaml("config/algorithm_config.yaml")
planner = ImomdPlanner(graph, config)
result = planner.run_for(0.5)
print(result.path, result.cost)
```

`ImomdPlanner.update_graph()` preserves valid RRT* branches, recomputes their
costs, prunes descendants of removed edges, and continues the search on new
traffic weights.

### V2X web demo

```bash
./demo/run.sh
# open http://127.0.0.1:8000
```

The UI provides a three-click source → waypoint → target wizard, separate
colors for both route legs, manual edge traffic, automatic V2X events, exact
route verification, and warm-start retention statistics. One backend scheduler
generates each V2X tick and broadcasts it to all connected pages.

See [demo/README.md](demo/README.md) and
[demo/BROWSER_USER_STORIES.md](demo/BROWSER_USER_STORIES.md).

## Correctness evidence

The project uses complementary checks rather than treating a successful demo
as proof:

- unit/integration tests for graph, tree, RRT*, ECI-Gen, baselines, OSM, config,
  traffic, and warm-start behavior;
- an independent Dijkstra + objective-permutation oracle that validates every
  returned edge, endpoint, objective, and reported path cost;
- a repeatable C++/Rust black-box harness for both original fake-map scenarios;
- Python package tests, strict Clippy, API smoke tests, and real browser user
  stories.

```bash
cargo test --all-targets
cargo test --all-targets -- --ignored
python scripts/compare_cpp_reference.py --build-cpp
python scripts/test_demo_api.py  # demo server must be running
```

On the reference fake maps, both C++ and Rust match the exact oracle. The
bugtrap map may choose different symmetric branches, but route cost and tree
size agree. This is strong regression evidence, not a formal proof of optimality
for every arbitrary graph or every finite anytime budget.

For the full evidence matrix and remaining limits, see
[docs/verification.md](docs/verification.md). The design contract is in
[docs/superpowers/specs/2026-07-13-imomd-rrtstar-design.md](docs/superpowers/specs/2026-07-13-imomd-rrtstar-design.md).
