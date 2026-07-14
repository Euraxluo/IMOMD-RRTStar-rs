const API = "";

let state = null;
let interactionMode = "route"; // route | traffic
let routeStep = "source"; // source | objective | target
let draftRoute = { source: null, objective: null, target: null };
let trafficPolygon = [];
let selectedTrafficEdges = [];
const adjacency = new Map();

const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
const statsEl = document.getElementById("stats");
const eventsEl = document.getElementById("events");
const mapHintEl = document.getElementById("map-hint");
const routeStepEl = document.getElementById("route-step");
const routeGuideEl = document.getElementById("route-guide");
const trafficGuideEl = document.getElementById("traffic-guide");
const selectedLaneCountEl = document.getElementById("selected-lane-count");

const STEP_TEXT = {
  source: "第 1 步：点击<strong>起点</strong>（绿色；点击道路附近会自动吸附）",
  objective: "第 2 步：点击<strong>途经点</strong>（橙色）",
  target: "第 3 步：点击<strong>终点</strong>（粉色），系统会自动规划",
  done: "已选完 3 个点，正在重规划…",
};

// Each destination leg has its own colour. The first two make the usual
// source -> objective -> target flow immediately legible on the map.
const PATH_LEG_COLORS = ["#00d4ff", "#f472b6", "#f59e0b", "#a78bfa"];

// Read-only diagnostics for browser user-story tests and live demos.
window.__IMOMD_DEMO__ = Object.freeze({ pathLegColors: [...PATH_LEG_COLORS] });

function nodePos(nodes, id, bounds) {
  const n = nodes.find((x) => x.id === id);
  if (!n) return { x: 0, y: 0 };
  const x = ((n.lon - bounds.minLon) / (bounds.maxLon - bounds.minLon || 1)) * (canvas.width - 40) + 20;
  const y = ((bounds.maxLat - n.lat) / (bounds.maxLat - bounds.minLat || 1)) * (canvas.height - 40) + 20;
  return { x, y };
}

function computeBounds(nodes) {
  const lats = nodes.map((n) => n.lat);
  const lons = nodes.map((n) => n.lon);
  return {
    minLat: Math.min(...lats),
    maxLat: Math.max(...lats),
    minLon: Math.min(...lons),
    maxLon: Math.max(...lons),
  };
}

function edgeColor(level) {
  switch (level) {
    case "slow": return "#f0ad4e";
    case "jam": return "#d9534f";
    case "blocked": return "#444";
    default: return "#5cb85c";
  }
}

function canvasPoint(ev) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return {
    x: (ev.clientX - rect.left) * scaleX,
    y: (ev.clientY - rect.top) * scaleY,
  };
}

function buildAdjacency(edges) {
  adjacency.clear();
  for (const e of edges) {
    // Keep blocked edges selectable so the user can restore one to `free`.
    if (!adjacency.has(e.from)) adjacency.set(e.from, new Set());
    adjacency.get(e.from).add(e.to);
    if (!adjacency.has(e.to)) adjacency.set(e.to, new Set());
    adjacency.get(e.to).add(e.from);
  }
}

function roleOfNode(id, dest) {
  if (id === dest.source) return "src";
  if (id === dest.target) return "tgt";
  if (dest.objectives.includes(id)) return "obj";
  return null;
}

function roleOfDraftNode(id) {
  if (id === draftRoute.source) return "src";
  if (id === draftRoute.objective) return "obj";
  if (id === draftRoute.target) return "tgt";
  return null;
}

function pointInPolygon(point, polygon) {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x;
    const yi = polygon[i].y;
    const xj = polygon[j].x;
    const yj = polygon[j].y;
    const intersects = yi > point.y !== yj > point.y
      && point.x < ((xj - xi) * (point.y - yi)) / (yj - yi || 1e-9) + xi;
    if (intersects) inside = !inside;
  }
  return inside;
}

function recomputeSelectedTrafficEdges() {
  if (!state || trafficPolygon.length < 3) {
    selectedTrafficEdges = [];
    updateTrafficUI();
    return;
  }
  const { nodes, edges } = state.view;
  const bounds = computeBounds(nodes);
  selectedTrafficEdges = edges.filter((edge) => {
    const a = nodePos(nodes, edge.from, bounds);
    const b = nodePos(nodes, edge.to, bounds);
    return pointInPolygon({ x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 }, trafficPolygon);
  });
  updateTrafficUI();
}

function drawTrafficPolygon() {
  if (trafficPolygon.length === 0) return;
  ctx.save();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#facc15";
  ctx.fillStyle = "rgba(250, 204, 21, 0.13)";
  ctx.setLineDash([8, 5]);
  ctx.beginPath();
  ctx.moveTo(trafficPolygon[0].x, trafficPolygon[0].y);
  for (const p of trafficPolygon.slice(1)) ctx.lineTo(p.x, p.y);
  if (trafficPolygon.length >= 3) ctx.closePath();
  ctx.stroke();
  if (trafficPolygon.length >= 3) ctx.fill();
  ctx.setLineDash([]);
  for (const p of trafficPolygon) {
    ctx.beginPath();
    ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
    ctx.fillStyle = "#facc15";
    ctx.fill();
    ctx.strokeStyle = "#fff7ad";
    ctx.stroke();
  }
  ctx.restore();
}

function drawMap(snapshot) {
  const view = snapshot.view;
  const nodes = view.nodes;
  const edges = view.edges;
  const path = snapshot.path || [];
  const dest = snapshot.destinations;
  const bounds = computeBounds(nodes);
  buildAdjacency(edges);
  const selectedKeys = new Set(selectedTrafficEdges.map((e) => `${Math.min(e.from, e.to)}-${Math.max(e.from, e.to)}`));

  ctx.fillStyle = "#0f1419";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  for (const e of edges) {
    const a = nodePos(nodes, e.from, bounds);
    const b = nodePos(nodes, e.to, bounds);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    const isSelected = interactionMode === "traffic"
      && selectedKeys.has(`${Math.min(e.from, e.to)}-${Math.max(e.from, e.to)}`);
    ctx.strokeStyle = isSelected ? "#facc15" : edgeColor(e.level);
    ctx.lineWidth = isSelected ? 6 : e.level === "jam" || e.level === "blocked" ? 4 : 2;
    if (!Number.isFinite(e.weight)) ctx.setLineDash([6, 4]);
    else ctx.setLineDash([]);
    ctx.stroke();
  }

  if (interactionMode === "traffic") drawTrafficPolygon();

  if (path.length > 1) {
    let legIndex = 0;
    for (let i = 1; i < path.length; i++) {
      const from = nodePos(nodes, path[i - 1], bounds);
      const to = nodePos(nodes, path[i], bounds);
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.strokeStyle = PATH_LEG_COLORS[Math.min(legIndex, PATH_LEG_COLORS.length - 1)];
      ctx.lineWidth = 5;
      ctx.setLineDash([]);
      ctx.stroke();

      // The edge entering an objective belongs to the preceding leg; all
      // following edges belong to the next leg.
      if (dest.objectives.includes(path[i])) legIndex += 1;
    }
  }

  const labels = [];
  const showDraft = interactionMode === "route" && routeStep !== "source";
  for (const n of nodes) {
    const p = nodePos(nodes, n.id, bounds);
    // Once a new route selection starts, hide the previous destination
    // markers and show only the three choices currently being made.
    const role = showDraft ? roleOfDraftNode(n.id) : roleOfNode(n.id, dest);
    const isPath = path.includes(n.id);

    let radius = 4;
    let fill = "#eee";
    let stroke = "#666";
    if (role === "src") {
      radius = 9; fill = "#4caf50"; stroke = "#fff";
      labels.push({ p, text: "起", color: "#4caf50" });
    } else if (role === "obj") {
      radius = 9; fill = "#ff9800"; stroke = "#fff";
      labels.push({ p, text: "经", color: "#ff9800" });
    } else if (role === "tgt") {
      radius = 9; fill = "#e91e63"; stroke = "#fff";
      labels.push({ p, text: "终", color: "#e91e63" });
    } else if (isPath) {
      radius = 6; fill = "#00d4ff";
    }

    ctx.beginPath();
    ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  ctx.font = "bold 11px system-ui";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (const { p, text, color } of labels) {
    ctx.fillStyle = color;
    ctx.fillText(text, p.x, p.y - 14);
  }
}

function setMode(mode) {
  interactionMode = mode;
  document.querySelectorAll(".mode-tabs .mode").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.mode === mode);
  });
  routeGuideEl.classList.toggle("hidden", mode !== "route");
  trafficGuideEl.classList.toggle("hidden", mode !== "traffic");
  updateMapHint();
  if (state) drawMap(state);
}

function updateMapHint() {
  if (!state) {
    mapHintEl.textContent = "加载中…";
    return;
  }
  if (interactionMode === "route") {
    mapHintEl.innerHTML = STEP_TEXT[routeStep] || "路线已选定";
  } else if (trafficPolygon.length < 3) {
    mapHintEl.textContent = "路况模式：点击地图围出一个区域（至少 3 个点）";
  } else {
    mapHintEl.textContent = `已选 ${selectedTrafficEdges.length} 条 lane，点击「应用路况」触发重规划`;
  }
}

function updateTrafficUI() {
  if (selectedLaneCountEl) {
    selectedLaneCountEl.textContent = `${selectedTrafficEdges.length} 条 lane`;
  }
  updateMapHint();
}

function updateRouteUI() {
  routeStepEl.innerHTML = STEP_TEXT[routeStep] || "✓ 路线已更新，查看地图上的蓝色路径";
  document.getElementById("badge-src").textContent =
    draftRoute.source != null ? `起点 #${draftRoute.source}` : "起点 —";
  document.getElementById("badge-obj").textContent =
    draftRoute.objective != null ? `途经 #${draftRoute.objective}` : "途经 —";
  document.getElementById("badge-tgt").textContent =
    draftRoute.target != null ? `终点 #${draftRoute.target}` : "终点 —";
  updateMapHint();
}

function syncFromServer(snapshot) {
  const dest = snapshot.destinations || {};
  draftRoute = {
    source: dest.source ?? null,
    objective: (dest.objectives && dest.objectives[0]) ?? null,
    target: dest.target ?? null,
  };
  routeStep = "source";
  document.getElementById("src").value = dest.source ?? "";
  document.getElementById("obj").value = draftRoute.objective ?? "";
  document.getElementById("tgt").value = dest.target ?? "";
  updateRouteUI();
}

function renderStats(snapshot) {
  const v = snapshot.verification || {};
  const verifyLine = v.ok
    ? `<div class="verify ok"><b>校验</b> ✓ ${v.message}</div>`
    : v.message
      ? `<div class="verify fail"><b>校验</b> ✗ ${v.message}</div>`
      : "";
  const update = snapshot.tree_update;
  const replanText = snapshot.replan_mode === "warm_start" && update
    ? `增量复用（保留 ${update.retained_tree_nodes}/${update.previous_tree_nodes} 个树节点）`
    : snapshot.replan_mode === "resume"
      ? "继续 anytime 搜索"
      : "新建搜索";
  statsEl.innerHTML = `
    <div><b>地图</b> ${snapshot.map_name} (${snapshot.node_count} 节点)</div>
    <div><b>路径代价</b> ${snapshot.cost != null ? snapshot.cost.toFixed(1) + " m" : "—"}</div>
    <div><b>Dijkstra 最优</b> ${v.oracle_cost != null ? v.oracle_cost.toFixed(1) + " m" : "—"}</div>
    <div><b>V2X 模拟</b> ${snapshot.auto_v2x ? "运行中" : "已停止"}</div>
    <div><b>重规划模式</b> ${replanText}</div>
    ${verifyLine}
  `;
  eventsEl.innerHTML = (snapshot.events || [])
    .slice()
    .reverse()
    .map((e) => `<li>${e}</li>`)
    .join("");
}

function renderMapSelect(snapshot) {
  const select = document.getElementById("map-select");
  if (!select || !snapshot.available_maps) return;
  const current = snapshot.map_key || "";
  const existing = [...select.options].map((option) => option.value).join(",");
  const next = snapshot.available_maps.map((map) => map.key).join(",");
  if (existing !== next) {
    select.innerHTML = snapshot.available_maps
      .map((map) => `<option value="${map.key}">${map.name}</option>`)
      .join("");
  }
  select.value = current;
}

async function fetchState() {
  const res = await fetch(`${API}/api/state`);
  state = await res.json();
  // Only a deliberate fetch (initial load or a completed command) resets the
  // route wizard. WebSocket heartbeats must not discard a user's in-progress
  // map clicks.
  syncFromServer(state);
  drawMap(state);
  renderStats(state);
  renderMapSelect(state);
}

async function postJson(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body == null ? "null" : JSON.stringify(body),
  });
  if (!res.ok) {
    const err = await res.text();
    alert(`操作失败：${err || res.statusText}\n\n建议点击「智能推荐路线」`);
    await fetchState();
    return null;
  }
  const data = await res.json();
  await fetchState();
  return data;
}

async function submitRoute(source, objective, target) {
  return postJson(`${API}/api/destinations`, {
    source,
    objectives: [objective],
    target,
  });
}

async function switchMap(mapKey) {
  const data = await postJson(`${API}/api/map`, { map_key: mapKey });
  trafficPolygon = [];
  selectedTrafficEdges = [];
  updateTrafficUI();
  return data;
}

function pickNodeAt(mx, my) {
  const nodes = state.view.nodes;
  const bounds = computeBounds(nodes);
  let best = null;
  let bestD = Infinity;
  for (const n of nodes) {
    const p = nodePos(nodes, n.id, bounds);
    const d = Math.hypot(p.x - mx, p.y - my);
    if (d < bestD) {
      bestD = d;
      best = n.id;
    }
  }
  // OSM nodes are visually dense; users should be able to click a road area
  // instead of having to hit a 4px marker precisely.
  const snapRadius = 56;
  return bestD <= snapRadius ? best : null;
}

canvas.addEventListener("click", async (ev) => {
  if (!state) return;
  const point = canvasPoint(ev);

  if (interactionMode === "route") {
    const nodeId = pickNodeAt(point.x, point.y);
    if (nodeId == null) return;
    if (routeStep === "source") {
      draftRoute = { source: nodeId, objective: null, target: null };
      routeStep = "objective";
    } else if (routeStep === "objective") {
      if (nodeId === draftRoute.source) {
        alert("途经点不能和起点相同");
        return;
      }
      draftRoute.objective = nodeId;
      routeStep = "target";
    } else if (routeStep === "target") {
      if (nodeId === draftRoute.source || nodeId === draftRoute.objective) {
        alert("终点不能和起点或途经点相同");
        return;
      }
      draftRoute.target = nodeId;
      routeStep = "done";
      updateRouteUI();
      drawMap(state);
      await submitRoute(draftRoute.source, draftRoute.objective, draftRoute.target);
      return;
    }
    updateRouteUI();
    drawMap(state);
    return;
  }

  // traffic mode
  trafficPolygon.push(point);
  recomputeSelectedTrafficEdges();
  drawMap(state);
});

document.getElementById("btn-smart").onclick = () =>
  postJson(`${API}/api/destinations/auto`, null);

document.getElementById("btn-reset-route").onclick = () => {
  routeStep = "source";
  draftRoute = { source: null, objective: null, target: null };
  updateRouteUI();
  if (state) drawMap(state);
};

document.getElementById("mode-route").onclick = () => setMode("route");
document.getElementById("mode-traffic").onclick = () => setMode("traffic");
document.getElementById("map-select").onchange = (ev) => switchMap(ev.target.value);

document.getElementById("btn-traffic-undo").onclick = () => {
  trafficPolygon.pop();
  recomputeSelectedTrafficEdges();
  if (state) drawMap(state);
};

document.getElementById("btn-traffic-reset").onclick = () => {
  trafficPolygon = [];
  selectedTrafficEdges = [];
  updateTrafficUI();
  if (state) drawMap(state);
};

document.getElementById("btn-traffic-apply").onclick = async () => {
  if (selectedTrafficEdges.length === 0) {
    alert("请先用至少 3 个点框选出 lane");
    return;
  }
  const level = document.getElementById("level").value;
  const edges = selectedTrafficEdges.map((edge) => ({ from: edge.from, to: edge.to }));
  await postJson(`${API}/api/traffic/edges`, { edges, level });
  trafficPolygon = [];
  selectedTrafficEdges = [];
  updateTrafficUI();
};

document.getElementById("btn-replan").onclick = () => postJson(`${API}/api/replan?seconds=1.5`, null);
document.getElementById("btn-clear").onclick = () => postJson(`${API}/api/traffic/clear`, null);
document.getElementById("btn-auto-on").onclick = () => postJson(`${API}/api/v2x/auto?enabled=true`, null);
document.getElementById("btn-auto-off").onclick = () => postJson(`${API}/api/v2x/auto?enabled=false`, null);

document.getElementById("btn-dest").onclick = () => {
  const source = Number(document.getElementById("src").value);
  const objectives = [Number(document.getElementById("obj").value)];
  const target = Number(document.getElementById("tgt").value);
  return submitRoute(source, objectives[0], target);
};

function connectWs() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/v2x`);
  ws.onmessage = (msg) => {
    const data = JSON.parse(msg.data);
    if (data.state) {
      state = data.state;
      if (data.result) {
        state.path = data.result.path;
        state.cost = data.result.cost;
      }
      drawMap(state);
      renderStats(state);
      renderMapSelect(state);
    }
  };
  ws.onclose = () => setTimeout(connectWs, 2000);
}

setMode("route");
fetchState().then(connectWs);
