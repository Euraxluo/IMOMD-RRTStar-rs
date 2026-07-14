# IMOMD-RRTStar-rs

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/Rust-2021-orange.svg)](Cargo.toml)
[![Python](https://img.shields.io/badge/Python-3.8%E2%80%933.13-blue.svg)](pyproject.toml)
[![arXiv](https://img.shields.io/badge/arXiv-2205.14853-b31b1b.svg)](https://arxiv.org/abs/2205.14853)

Rust reproduction of **Informable Multi-Objective and Multi-Directional RRT\***
(IMOMD-RRT\*), an anytime multi-objective path planner that jointly grows
multi-directional RRT\* trees and refines destination visit order.

| Resource | Link |
|---|---|
| Original paper | [arXiv:2205.14853](https://arxiv.org/abs/2205.14853) · [ICRA 2023](https://doi.org/10.1109/ICRA48891.2023.10160838) |
| Reference C++ | [UMich-BipedLab/IMOMD-RRTStar](https://github.com/UMich-BipedLab/IMOMD-RRTStar/tree/lib_isrr_release) |
| This repository | [Euraxluo/IMOMD-RRTStar-rs](https://github.com/Euraxluo/IMOMD-RRTStar-rs) |

The implementation preserves the paper / C++ algorithmic structure while
exposing a pause/resume Rust API, typed Python bindings, OSM loaders, a
pluggable realtime navigation session, and a FastAPI V2X demonstration.

> **Scope.** This is an independent open-source reproduction for research and
> teaching. It is not affiliated with the University of Michigan Biped Lab.
> Please cite the original authors when referring to the algorithm.

## Features

| Area | Status |
|---|---|
| Multi-tree IMOMD-RRT\*, rewiring, pseudo-objective mode | Implemented |
| ECI-Gen (insertion, swapping, shortcut, genetic refinement) | Implemented |
| Bi-A\* and ANA\* baselines | Implemented |
| Fake, custom YAML, and OSM maps | Implemented |
| Original-style YAML configuration and CLI | Implemented |
| Pause/resume anytime API and dynamic graph warm-start | Implemented |
| Pluggable `NavigationSession` with greedy / exact / IMOMD race | Implemented |
| PyO3 / maturin package with PEP 561 types | Implemented |
| FastAPI + Canvas V2X demo (anytime morph, traffic, ego replan) | Implemented |
| ROS wrapper | Out of scope |

## Quick start

### Rust

```bash
cargo test --all-targets
cargo run -- --config config/algorithm_config.yaml
```

### Python

Supported CPython versions: **3.8–3.13**.

```bash
python -m venv .venv
.venv/bin/pip install maturin
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  .venv/bin/maturin develop --release --features python,extension-module
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

### Interactive V2X demo

```bash
./demo/run.sh
# open http://127.0.0.1:8000
```

The demo races a greedy tour, an exact Dijkstra+TSP lane (≤8 waypoints), and
IMOMD anytime search: the first feasible path is shown, then only strictly
better costs replace it. See [demo/README.md](demo/README.md) and
[docs/navigation-plugins.md](docs/navigation-plugins.md).

## Reproducing experiments

```bash
cargo test --all-targets
cargo test --all-targets -- --ignored          # larger / optional cases
python scripts/compare_cpp_reference.py --build-cpp
python scripts/test_demo_api.py                # demo server must be running
```

Optional Chicago downtown OSM for the mega-city demo:

```bash
python scripts/download_chicago_osm.py
DEMO_MAP=chicago_osm ./demo/run.sh
```

Correctness evidence (oracle, C++ harness, limits) is documented in
[docs/verification.md](docs/verification.md). Design notes live under
[docs/superpowers/specs/](docs/superpowers/specs/).

## Project layout

```
src/                 Rust core (RRT*, RTSP, baselines, navigation session)
python/              Typed Python package surface
config/              Algorithm / OSM YAML configs
demo/                FastAPI + Canvas V2X demonstration
scripts/             C++ comparison, OSM download, API smoke tests
test/                Python unit tests
tests/               Rust integration / oracle tests
docs/                Verification notes, plugin docs, design specs
```

## Citation

If you use the **IMOMD-RRT\*** algorithm, cite the original paper:

```bibtex
@inproceedings{huang2023imomd,
  title     = {Informable Multi-Objective and Multi-Directional {RRT*} System for Robot Path Planning},
  author    = {Huang, Jiunn-Kai and Tan, Yingwen and Lee, Dongmyeong and Desaraju, Vishnu R. and Grizzle, Jessy W.},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2023},
  doi       = {10.1109/ICRA48891.2023.10160838},
  note      = {arXiv:2205.14853}
}
```

If you use **this Rust/Python reproduction**, please also cite the software
(see [`CITATION.cff`](CITATION.cff)):

```bibtex
@software{euraxluo_imomd_rrtstar_rs,
  title        = {{IMOMD-RRTStar-rs}: A Rust Reproduction of Informable Multi-Objective and Multi-Directional {RRT*}},
  author       = {Euraxluo},
  year         = {2026},
  url          = {https://github.com/Euraxluo/IMOMD-RRTStar-rs},
  license      = {Apache-2.0}
}
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for setup, testing, and pull-request
expectations.

## License

This project is released under the [Apache License 2.0](LICENSE).
The original algorithm and C++ reference remain the work of their respective
authors; this repository only redistributes an independent reimplementation.
