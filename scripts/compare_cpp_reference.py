#!/usr/bin/env python3
"""Repeatable black-box differential check against the original C++ release.

The harness runs both binaries with isolated, generated fake-map configs, then
checks each route against an independent Dijkstra/permutation oracle.  It never
edits the reference checkout's config files.
"""

from __future__ import annotations

import argparse
import heapq
import itertools
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
NUMBER = r"(?:[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?|inf)"


@dataclass(frozen=True)
class Scenario:
    name: str
    map_type: int
    source: int
    objectives: tuple[int, ...]
    target: int


@dataclass
class RunResult:
    implementation: str
    scenario: str
    path: list[int]
    cost: float
    tree_size: int


SCENARIOS = {
    "fake_map_1": Scenario("fake_map_1", -1, 0, (1,), 2),
    "bugtrap": Scenario("bugtrap", -2, 6, (2,), 0),
}


def fake_graph(map_type: int) -> tuple[list[tuple[float, float]], list[set[int]]]:
    if map_type == -1:
        nodes = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (1.0, -1.0)]
        edges = [{1, 3}, {0, 2, 3}, {1}, {0, 1}]
    elif map_type == -2:
        nodes = [
            (-0.1, 2.1),
            (0.1, 2.1),
            (0.0, 2.0),
            (-1.0, 0.0),
            (1.0, 0.0),
            (0.0, -1.0),
            (0.0, -1.1),
        ]
        edges = [{1, 2}, {0, 2}, {0, 1, 3, 4}, {2, 5}, {2, 5}, {3, 4, 6}, {5}]
    else:
        raise ValueError(f"unsupported fake map: {map_type}")
    return nodes, edges


def haversine(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    value = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 6_371_000.0 * 2.0 * math.atan2(math.sqrt(value), math.sqrt(1.0 - value))


def edge_weight(map_type: int, left: int, right: int) -> float | None:
    nodes, edges = fake_graph(map_type)
    if right not in edges[left]:
        return None
    return haversine(nodes[left], nodes[right])


def dijkstra(map_type: int, source: int, target: int) -> float:
    nodes, edges = fake_graph(map_type)
    distances = [math.inf] * len(nodes)
    distances[source] = 0.0
    queue = [(0.0, source)]
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != distances[node]:
            continue
        if node == target:
            return cost
        for neighbor in edges[node]:
            candidate = cost + haversine(nodes[node], nodes[neighbor])
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return math.inf


def oracle_cost(scenario: Scenario) -> float:
    best = math.inf
    for order in itertools.permutations(scenario.objectives):
        waypoints = (scenario.source, *order, scenario.target)
        cost = sum(
            dijkstra(scenario.map_type, left, right)
            for left, right in zip(waypoints, waypoints[1:])
        )
        best = min(best, cost)
    return best


def config_text(scenario: Scenario) -> str:
    objectives = ", ".join(str(item) for item in scenario.objectives)
    return f"""general:
  system: 0
  pseudo: 0
  log_data: 0
  print_path: 1
  max_iter: 50000
  max_time: 2
rrt_params:
  goal_bias: 1.0
  random_seed: 0
destinations:
  source_id: {scenario.source}
  objective_ids: [{objectives}]
  target_id: {scenario.target}
map:
  type: {scenario.map_type}
  path: ""
  name: ""
rtsp_settings:
  shortcut: 1
  swapping: 1
  genetic: 0
  ga:
    random_seed: 0
    mutation_iter: 10
    population: 10
    generation: 1
"""


def last_number(pattern: str, output: str) -> float:
    values = [float(value) for value in re.findall(pattern, output, flags=re.IGNORECASE)]
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        raise RuntimeError(f"could not parse finite value using {pattern!r}")
    return finite[-1]


def last_path(output: str) -> list[int]:
    candidates: list[list[int]] = []
    for line in output.splitlines():
        if "->" not in line or "#" not in line:
            continue
        values = [int(value) for value in re.findall(r"\d+(?=\s*->)", line)]
        if values:
            candidates.append(values)
    if not candidates:
        raise RuntimeError("could not parse a printed path")
    return candidates[-1]


def run_process(command: list[str], cwd: Path, timeout: float) -> str:
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    output = ANSI.sub("", completed.stdout)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n{output[-4000:]}"
        )
    return output


def run_cpp(binary: Path, scenario: Scenario) -> RunResult:
    with tempfile.TemporaryDirectory(prefix="imomd-cpp-diff-") as raw_tmp:
        tmp = Path(raw_tmp)
        config = tmp / "config"
        config.mkdir()
        (config / "algorithm_config.yaml").write_text(config_text(scenario), encoding="utf-8")
        isolated_binary = tmp / "imomd-reference"
        shutil.copy2(binary, isolated_binary)
        output = run_process([str(isolated_binary)], tmp, timeout=15.0)
    return RunResult(
        implementation="cpp",
        scenario=scenario.name,
        path=last_path(output),
        cost=last_number(rf"Path Cost\[m\]:\s*({NUMBER})", output),
        tree_size=round(last_number(r"Tree Size:\s*(\d+)", output)),
    )


def run_rust(binary: Path, scenario: Scenario) -> RunResult:
    with tempfile.TemporaryDirectory(prefix="imomd-rust-diff-") as raw_tmp:
        config = Path(raw_tmp) / "algorithm_config.yaml"
        config.write_text(config_text(scenario), encoding="utf-8")
        output = run_process([str(binary), "--config", str(config)], ROOT, timeout=15.0)
    return RunResult(
        implementation="rust",
        scenario=scenario.name,
        path=last_path(output),
        cost=last_number(rf"Path cost\[m\]:\s*({NUMBER})", output),
        tree_size=round(last_number(r"Tree size:\s*(\d+)", output)),
    )


def verify_result(result: RunResult, scenario: Scenario, oracle: float) -> None:
    if result.path[0] != scenario.source or result.path[-1] != scenario.target:
        raise AssertionError(f"{result.implementation} endpoints invalid: {result.path}")
    missing = set(scenario.objectives).difference(result.path)
    if missing:
        raise AssertionError(f"{result.implementation} skipped objectives {sorted(missing)}")
    recomputed = 0.0
    for left, right in zip(result.path, result.path[1:]):
        weight = edge_weight(scenario.map_type, left, right)
        if weight is None:
            raise AssertionError(f"{result.implementation} path contains non-edge {left}->{right}")
        recomputed += weight
    # The reference executable prints with six significant digits, so its
    # large fake-map costs are rounded to sub-meter precision.
    if not math.isclose(result.cost, recomputed, abs_tol=1.0):
        raise AssertionError(
            f"{result.implementation} reported {result.cost}, recomputed {recomputed}"
        )
    if not math.isclose(result.cost, oracle, abs_tol=1.0):
        raise AssertionError(f"{result.implementation} cost {result.cost} != oracle {oracle}")


def ensure_binary(path: Path, build_command: list[str] | None, cwd: Path) -> None:
    if path.is_file():
        return
    if build_command is None:
        raise FileNotFoundError(f"binary not found: {path}")
    subprocess.run(build_command, cwd=cwd, check=True)
    if not path.is_file():
        raise FileNotFoundError(f"build did not create binary: {path}")


def build_cpp_reference(output: Path) -> None:
    reference = ROOT / "tmp/imomd-cpp"
    output.parent.mkdir(parents=True, exist_ok=True)
    sources = [
        "main.cpp",
        "src/imomd_rrt_star.cpp",
        "src/eci_gen_tsp_solver.cpp",
        "src/osm_parser.cpp",
        "src/baseline/bi_a_star.cpp",
        "src/baseline/ana_star.cpp",
        "src/tinyxml2/tinyxml2.cpp",
    ]
    subprocess.run(
        [
            "g++",
            "-Wall",
            "-Wextra",
            "-Wno-comment",
            "-O3",
            "-std=c++11",
            "-Iinclude",
            *sources,
            "-lpthread",
            "-o",
            str(output),
        ],
        cwd=reference,
        check=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=["all", *SCENARIOS], default="all")
    parser.add_argument("--cpp-binary", type=Path)
    parser.add_argument(
        "--rust-binary", type=Path, default=ROOT / "target/debug/main"
    )
    parser.add_argument("--build-cpp", action="store_true")
    parser.add_argument("--build-rust", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cpp_binary = (
        args.cpp_binary
        or (
            ROOT / "target/cpp-reference/imomd-reference"
            if args.build_cpp
            else ROOT / "tmp/imomd-cpp/main"
        )
    ).resolve()
    rust_binary = args.rust_binary.resolve()
    try:
        if args.build_cpp:
            build_cpp_reference(cpp_binary)
        else:
            ensure_binary(cpp_binary, None, ROOT / "tmp/imomd-cpp")
        ensure_binary(
            rust_binary,
            ["cargo", "build", "--bin", "main"] if args.build_rust else None,
            ROOT,
        )
        selected = SCENARIOS.values() if args.scenario == "all" else [SCENARIOS[args.scenario]]
        report: list[dict[str, object]] = []
        for scenario in selected:
            oracle = oracle_cost(scenario)
            cpp = run_cpp(cpp_binary, scenario)
            rust = run_rust(rust_binary, scenario)
            verify_result(cpp, scenario, oracle)
            verify_result(rust, scenario, oracle)
            if not math.isclose(cpp.cost, rust.cost, abs_tol=1.0):
                raise AssertionError(
                    f"{scenario.name}: C++ {cpp.cost} and Rust {rust.cost} differ"
                )
            report.append(
                {
                    "scenario": asdict(scenario),
                    "oracle_cost": oracle,
                    "cpp": asdict(cpp),
                    "rust": asdict(rust),
                }
            )
        if args.json:
            print(json.dumps(report, indent=2))
        else:
            for item in report:
                cpp, rust = item["cpp"], item["rust"]
                print(
                    f"{item['scenario']['name']}: oracle={item['oracle_cost']:.4f}m "
                    f"C++={cpp['cost']:.4f} {cpp['path']} "
                    f"Rust={rust['cost']:.4f} {rust['path']}"
                )
            print("C++/Rust differential check: OK")
        return 0
    except (
        AssertionError,
        FileNotFoundError,
        OSError,
        RuntimeError,
        subprocess.SubprocessError,
    ) as exc:
        print(f"differential check failed: {exc}", file=sys.stderr)
        if isinstance(exc, OSError) and not args.build_cpp:
            print("hint: rebuild the reference for this host with --build-cpp", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
