# In-app Browser 用户故事

前置：运行 `./demo/run.sh`，在 Codex in-app browser 打开
`http://127.0.0.1:8000/`。

## US-01 初始加载与 oracle

1. 标题包含 `IMOMD-RRT*`；
2. 地图、起点/途经/终点和两段路线可见；
3. 状态显示 `校验 ✓`；
4. 路径代价不低于 Dijkstra 最优，通常在当前小预算下相等。

通过：`GET /api/state` 的 `verification.ok === true`。

## US-02 三次点击完成路线

1. 点击“重新选点”；
2. 在地图道路附近点击一次，提示变为“第 2 步：点击途经点”；
3. 等待至少 3 秒（跨过一个 WebSocket heartbeat），步骤和起点 badge 不应重置；
4. 点击途经点，提示变为第 3 步；
5. 点击终点，先显示“正在重规划”，完成后恢复第 1 步；
6. 新的三个 badge、路径代价和 `校验 ✓` 同时更新。

通过：无需输入 node id；三次点击严格对应 source/objective/target。

## US-03 两段路线颜色

观察 objective 两侧：

- source → objective 为青色 `#00d4ff`；
- objective → target 为粉色 `#f472b6`。

通过：两段颜色在地图和图例中都不同，objective 节点为橙色。

## US-04 手动拥堵与恢复

1. 切换“设置路况”；
2. 选择“拥堵 ×5”；
3. 点击同一条道路的两个相邻 node；
4. 事件列表出现 `Edge a-b → jam`；
5. 状态显示“增量复用（保留 x/y 个树节点）”且 `校验 ✓`；
6. 对同一边选择“恢复畅通”，或点击“清除路况”；
7. 成本回落，事件显示 `free`/`Traffic cleared`，oracle 再次对齐。

通过：`replan_mode === "warm_start"`，`retained_tree_nodes > 0`，
`verification.path_valid === true`。

## US-05 多页面 V2X 广播

1. 可同时打开两个 demo 页面；
2. 记录 `/api/state` 的 `v2x_tick`；
3. 点击“开启 V2X 模拟”，等待一个 tick；
4. 两个页面应收到相同事件，tick 只增加 1，而不是按页面数增加；
5. 状态显示“V2X 模拟 运行中”“增量复用”和 `校验 ✓`；
6. 点击“停止 V2X”，再清除路况。

通过：单一服务端调度器只应用一次路况，再向所有 WebSocket 客户端广播。

## 等价 API 验收

```bash
python scripts/test_demo_api.py
```

该脚本验证 initial/oracle、warm-start、路径合法性、清除恢复和单调度器 tick。
