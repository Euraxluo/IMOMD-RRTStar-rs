# IMOMD-RRT* V2X Demo

FastAPI + Canvas 可视化：在 OSM 路网上叠加实时路权，并让同一个 Rust
IMOMD-RRT* planner 增量重规划。

## 启动

```bash
# 先在项目根目录构建 Python 扩展
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  .venv/bin/maturin develop --features python,extension-module

./demo/run.sh
# http://127.0.0.1:8000
```

默认地图为 `tmp/imomd-cpp/osm_data/FRB2.osm`。可覆盖：

```bash
DEMO_OSM_PATH=tmp/imomd-cpp/osm_data/quincy.osm ./demo/run.sh
```

若 OSM 文件不存在，服务会回退到 C++ bugtrap fake map。

## 最简单的交互方式

1. 保持“点地图选路线”模式；
2. 点击道路附近作为起点（绿）；
3. 再点击途经点（橙）；
4. 再点击终点（粉），系统自动规划。

点击会在较大半径内自动吸附最近 node，不需要记 node id。选择过程中 WebSocket
heartbeat 不会重置进度。旧 destination marker 会在新选择开始后隐藏，避免新旧
点混在一起。路线分段显示：

- source → objective：青色；
- objective → target：粉色。

也可点击“智能推荐路线”直接生成一条分散且可达的路线。

## 路况与 V2X

切换到“设置路况”，选择等级，再点击两个相邻 node：

| 等级 | 权重 |
|---|---|
| `free` | ×1，恢复基础权重 |
| `slow` | ×2.5 |
| `jam` | ×5 |
| `blocked` | 移除边 |

blocked 边仍保留在可视化 adjacency 中，因此可以再次选中并恢复为 free。
自动 V2X 每 3 秒产生一个 zone event。后台只有一个调度器；无论打开多少页面，
每个 tick 只修改一次图和 planner，然后向全部客户端广播同一状态。

路权更新采用 warm-start：保留仍合法的 RRT* 分支、重算 cost、剪除封路边的
descendants，再重建连接/RTSP 状态。只有 destinations 改变才创建新 planner。

## 页面上的正确性信号

每个 snapshot 都运行独立验证：

- 路径端点和 objectives；
- 每条 path edge 是否存在；
- 显示 cost 是否等于边权求和；
- Dijkstra + objective permutation 精确 oracle。

`校验 ✓` 表示当前路径满足上述契约。“Dijkstra 最优”是当前动态图的 oracle；
IMOMD-RRT* 是 anytime 算法，允许暂时高于 oracle，但不能低于 oracle。

## API

- `GET /api/state` — 地图、路径、验证、事件、`v2x_tick`、warm-start 统计
- `GET /api/verify` — 当前路径的独立验证报告
- `POST /api/destinations` — `{source, objectives, target}`
- `POST /api/destinations/auto`
- `POST /api/traffic/edge` — `{from, to, level}`
- `POST /api/traffic/zone` — `{nodes, level}`
- `POST /api/traffic/clear`
- `POST /api/replan?seconds=1.5`
- `POST /api/v2x/auto?enabled=true|false`
- `WS /ws/v2x` — 单调度器广播

无效 node/非相邻 edge 返回 400；当前路况使路线不可达时返回 409，页面不会继续
显示一条针对旧图的过期路径。

## 测试

```bash
# 服务运行时
python scripts/test_demo_api.py

# 可选 Playwright 自动化
uv sync --project demo --group dev
uv run --project demo --group dev playwright install chromium
uv run --project demo --group dev pytest demo/tests -v
```

可选 Playwright 测试位于 `demo/tests/test_user_stories.py`；算法 oracle 的纯 Python
单测位于 `demo/tests/test_verify.py`。
