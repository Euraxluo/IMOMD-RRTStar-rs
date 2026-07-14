# 实现正确性与验收指南

本文档定义 IMOMD-RRT* Rust 复现的证据链。目标不是用一个 demo 声称“绝对
正确”，而是分别验证实现结构、路径契约、参考实现一致性、动态更新和包装层。

## 1. 发布门禁

```bash
cargo fmt --check
cargo test --all-targets
cargo test --all-targets -- --ignored
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  cargo clippy --all-targets --all-features -- -D warnings

PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  maturin develop --features python,extension-module
python -m unittest discover test -v

cargo build --release
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  maturin build --release --features python,extension-module
```

Rust CLI 与 Python extension 是两个链接目标：前者用 Cargo 默认 features，后者用
maturin 驱动 `extension-module` feature；不要把 macOS 的扩展链接模式误当成普通
Rust binary 的 `cargo build --all-features`。

覆盖范围包括 geo、邻接图校验、fake/custom/OSM loader、RRT 树、IMOMD-RRT*、
ECI-Gen、pseudo 模式、Bi-A*、ANA*、配置、实验日志、traffic overlay 和
warm-start。Seattle 测试显式标为 ignored，因为原始 Seattle OSM 不是仓库资产；
存在数据时该测试会真实执行，而不是无条件跳过结果断言。

## 2. 精确 oracle 与路径契约

```bash
cargo test --test correctness_oracle
```

`tests/correctness_oracle.rs` 使用独立实现的 Dijkstra，并对 objectives 的所有
排列求精确 multi-objective 最优值。每个规划结果必须满足：

1. 第一个节点等于 source，最后一个节点等于 target；
2. 路径包含全部 objective；
3. 相邻节点都是当前图中的真实边；
4. `PlanningResult.cost` 等于路径边权之和；
5. 结果不能低于精确 oracle；
6. 在确定性的 fake map 场景中，结果必须等于 oracle。

该测试同时覆盖普通/pseudo 单多目标、GA 序列完整性，以及路权变化后保留树枝、
封路剪枝和重新规划。

## 3. C++ 黑盒差分

本地存在 `tmp/imomd-cpp` 原始 `lib_isrr_release` 源码时，执行：

```bash
python scripts/compare_cpp_reference.py --build-cpp
```

harness 会：

- 为当前主机从原始 C++ 源码构建独立参考二进制；
- 在临时目录生成配置，不修改参考仓库；
- 对 C++ 与 Rust 分别运行 fake_map_1 和 bugtrap；
- 用第三份独立 Dijkstra/permutation oracle 重算路径和成本；
- 校验端点、objectives、每条边、打印成本及 C++/Rust 差值。

当前基线：

| 场景 | C++ | Rust | 结论 |
|---|---|---|---|
| `0 → [1] → 2` | `0→1→2`, 约 444,780m | `0→1→2`, 444,779.7066m | 同一路径/同成本 |
| `6 → [2] → 0` | `6→5→4→2→0`, 约 432,724m | `6→5→3→2→0`, 432,723.5221m | 对称等价分支/同成本 |

C++ 默认流输出只有约六位有效数字，因此 harness 对其使用 1m 绝对打印容差；
Rust 内部和 oracle 的比较仍保留更高精度。

## 4. Anytime 与动态路权

Rust 的调用者 deadline 是暂停点，不会仅因为一个短时间片结束就永久终止搜索。
算法在达到配置上限或所有 destination trees 已完成且已有合法解时才置为 finished。
`StepStatus` 根据调用前后的真实连通性/最佳代价返回 `Connected`、
`PathImproved`、`Expanded` 或 `Finished`。

`update_graph()` 的不变量：

- node 数量、id 和坐标不能变化；
- 权重变化保留合法父子边并重新累计 cost；
- 被移除边的所有下游 descendants 被剪除；
- expandables、连接矩阵、并查集和 RTSP 解全部重建；
- `GraphUpdateStats` 报告更新前、保留和剪除的树节点数。

V2X API smoke 会断言 `replan_mode == "warm_start"` 且
`retained_tree_nodes > 0`：

```bash
./demo/run.sh
python scripts/test_demo_api.py
```

## 5. OSM 与可复现性

OSM node 按原始 OSM id 的稳定顺序映射到内部 `NodeId`。这避免了随机 seed 的
`HashSet` 遍历导致同一地图在不同进程中节点编号变化。固定算法 seed、相同输入
和相同平台调度下可重复；C++ `unordered_map` 与 Rust `FxHashMap` 的遍历/RNG
仍不要求逐次迭代 bit-for-bit 相同。

```bash
cargo test --test integration_osm
cargo test --test integration_large_scale
```

## 6. Python 包装与 wheel

Python 包公开 fake/custom/OSM map、traffic、config、planner、result 和更新统计。
长时间 `run_for`、`run_until`、`step`、`update_graph` 使用 PyO3
`Python::allow_threads`，因此 Rust/Rayon 工作期间不占用 Python GIL。

类型包采用 PEP 561 的 `__init__.pyi` + `py.typed`；构建 wheel 后应确认两者在包内：

```bash
maturin build --features python,extension-module
python -m unittest discover test -v
```

## 7. FastAPI / V2X / 浏览器

服务端只有一个 lifespan V2X 调度任务。每个 tick 只修改一次 traffic/planner，
再把同一 snapshot 广播给全部 WebSocket 客户端；页面数量不会放大事件频率。
规划操作由 `asyncio.Lock` 串行化并放到 worker thread，避免多个 HTTP/WebSocket
请求同时可变借用同一个 planner。

`scripts/test_demo_api.py` 验证：

- 初始路径和 oracle；
- zone 路况触发 warm-start；
- 更新后路径/成本仍合法；
- 清除路况恢复；
- 多客户端架构下后台 tick 只增加一次。

真实页面验收按 [demo/BROWSER_USER_STORIES.md](../demo/BROWSER_USER_STORIES.md)：

- 三次点击依次选择起点、途经、终点；
- WebSocket heartbeat 不清空未完成选择；
- 起点→途经为青色、途经→终点为粉色；
- 相邻边 jam 后成本变化且显示“增量复用”；
- 恢复/清除后 oracle 再次对齐；
- V2X 广播后仍显示 `校验 ✓`。

## 8. 可以和不可以据此声称什么

可以声称：核心结构与原始源码对应；参考小图输出与 C++/精确 oracle 一致；
返回路径满足可执行契约；动态路权会复用并修复搜索树；Python/Web 调用的是同一
Rust 核心而不是另写一个替代算法。

不能声称：对任意图、任意 objective 数量和任意有限时间预算都有形式化最优性
证明；与 C++ 每一次随机扩展 bit-for-bit 相同；未提供的 Seattle 全图已经在本机
跑过；模拟 V2X 等同于接入真实车端/路侧协议。ROS wrapper 也明确不在本项目范围。
