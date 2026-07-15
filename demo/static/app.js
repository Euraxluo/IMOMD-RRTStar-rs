const API = "";

let state = null;
let interactionMode = "route"; // route | traffic | ego
let routeStep = "source"; // source | objective | target | done
let draftRoute = { source: null, objectives: [], target: null };
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
  source: "Step 1: click the <strong>start</strong> (green; snaps to nearest road node)",
  objective:
    "Step 2: add <strong>waypoints</strong> (orange, multiple OK), then press “Waypoints done — pick goal”",
  target: "Step 3: click the <strong>goal</strong> (pink); planning starts automatically",
  done: "Selection complete — replanning…",
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
  if ((draftRoute.objectives || []).includes(id)) return "obj";
  if (id === draftRoute.target) return "tgt";
  return null;
}

function draftObjectiveIndex(id) {
  return (draftRoute.objectives || []).indexOf(id);
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
  const huge = nodes.length > 1200;
  // LOD: keep traffic / selected edges, subsample free roads on mega maps.
  const freeStride = huge ? Math.max(1, Math.ceil(edges.length / 5500)) : 1;

  ctx.fillStyle = "#060a12";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  let drawn = 0;
  for (let idx = 0; idx < edges.length; idx++) {
    const e = edges[idx];
    const key = `${Math.min(e.from, e.to)}-${Math.max(e.from, e.to)}`;
    const isSelected = interactionMode === "traffic" && selectedKeys.has(key);
    const interesting = e.level !== "free" || isSelected || !Number.isFinite(e.weight);
    if (!interesting && idx % freeStride !== 0) continue;
    const a = nodePos(nodes, e.from, bounds);
    const b = nodePos(nodes, e.to, bounds);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.strokeStyle = isSelected ? "#facc15" : edgeColor(e.level);
    ctx.lineWidth = isSelected ? 5 : e.level === "jam" || e.level === "blocked" ? 3.5 : huge ? 1.1 : 2;
    if (!Number.isFinite(e.weight)) ctx.setLineDash([6, 4]);
    else ctx.setLineDash([]);
    ctx.stroke();
    drawn += 1;
  }
  // Stash for stats debugging.
  snapshot._drawn_edges = drawn;

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
      ctx.lineWidth = huge ? 3.5 : 5;
      ctx.setLineDash([]);
      ctx.stroke();

      // The edge entering an objective belongs to the preceding leg; all
      // following edges belong to the next leg.
      if (dest.objectives.includes(path[i])) legIndex += 1;
    }
  }

  const labels = [];
  const showDraft = interactionMode === "route" && routeStep !== "source";
  const drawAllNodes = !huge;
  for (const n of nodes) {
    const role = showDraft ? roleOfDraftNode(n.id) : roleOfNode(n.id, dest);
    const isPath = path.includes(n.id);
    const isEgo = snapshot.ego_node === n.id;
    if (!drawAllNodes && !role && !isPath && !isEgo) continue;

    const p = nodePos(nodes, n.id, bounds);
    let radius = huge ? 2.2 : 4;
    let fill = "#eee";
    let stroke = "#666";
    if (role === "src") {
      radius = 9; fill = "#4caf50"; stroke = "#fff";
      labels.push({ p, text: "S", color: "#4caf50" });
    } else if (role === "obj") {
      radius = 9; fill = "#ff9800"; stroke = "#fff";
      const objs = showDraft
        ? draftRoute.objectives || []
        : dest.objectives || [];
      const idx = objs.indexOf(n.id);
      labels.push({
        p,
        text: objs.length > 1 && idx >= 0 ? `W${idx + 1}` : "W",
        color: "#ff9800",
      });
    } else if (role === "tgt") {
      radius = 9; fill = "#e91e63"; stroke = "#fff";
      labels.push({ p, text: "G", color: "#e91e63" });
    } else if (isPath) {
      radius = huge ? 3 : 6; fill = "#00d4ff";
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

  if (snapshot.ego_node != null) {
    const ego = nodePos(nodes, snapshot.ego_node, bounds);
    ctx.beginPath();
    ctx.arc(ego.x, ego.y, 11, 0, Math.PI * 2);
    ctx.strokeStyle = "#fbbf24";
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.fillStyle = "#fbbf24";
    ctx.font = "bold 12px system-ui";
    ctx.fillText("EGO", ego.x, ego.y - 16);
  }
}

function drawCostChart(snapshot) {
  const chart = document.getElementById("cost-chart");
  if (!chart) return;
  const c = chart.getContext("2d");
  const history = snapshot.cost_history || [];
  c.clearRect(0, 0, chart.width, chart.height);
  c.fillStyle = "#111827";
  c.fillRect(0, 0, chart.width, chart.height);
  c.fillStyle = "#9ca3af";
  c.font = "11px system-ui";
  c.fillText("Cost over time (anytime)", 8, 14);
  if (history.length < 2) {
    c.fillText("Waiting for path improvements…", 8, 48);
    return;
  }
  const costs = history.map((h) => h.cost).filter((x) => Number.isFinite(x));
  const min = Math.min(...costs);
  const max = Math.max(...costs);
  const span = Math.max(max - min, 1);
  c.beginPath();
  costs.forEach((cost, i) => {
    const x = 10 + (i / (costs.length - 1)) * (chart.width - 20);
    const y = chart.height - 12 - ((cost - min) / span) * (chart.height - 28);
    if (i === 0) c.moveTo(x, y);
    else c.lineTo(x, y);
  });
  c.strokeStyle = "#34d399";
  c.lineWidth = 2;
  c.stroke();
  const last = costs[costs.length - 1];
  c.fillStyle = "#34d399";
  c.fillText(`${last.toFixed(0)} m`, chart.width - 72, 14);
}

function setMode(mode) {
  interactionMode = mode;
  document.querySelectorAll(".mode").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.mode === mode);
  });
  routeGuideEl.classList.toggle("hidden", mode !== "route");
  trafficGuideEl.classList.toggle("hidden", mode !== "traffic");
  updateMapHint();
  if (state) drawMap(state);
}

function updateMapHint() {
  if (!state) {
    mapHintEl.textContent = "Loading…";
    return;
  }
  if (interactionMode === "route") {
    mapHintEl.innerHTML = STEP_TEXT[routeStep] || "Route selected";
  } else if (interactionMode === "ego") {
    mapHintEl.textContent = "Ego mode: click a map node to replan the remaining trip from there";
  } else if (trafficPolygon.length < 3) {
    mapHintEl.textContent = "Traffic mode: click the map to outline a region (≥3 points)";
  } else {
    mapHintEl.textContent = `${selectedTrafficEdges.length} lanes selected — click Apply to replan`;
  }
}

function updateTrafficUI() {
  if (selectedLaneCountEl) {
    selectedLaneCountEl.textContent = `${selectedTrafficEdges.length} lanes`;
  }
  updateMapHint();
}

function updateRouteUI() {
  const objs = draftRoute.objectives || [];
  const objField = document.getElementById("obj");
  if (objField && document.activeElement !== objField) {
    objField.value = objs.join(",");
  }
  const objLabel = objs.length
    ? `Waypoints ${objs.map((x) => `#${x}`).join(" ")}`
    : "Waypoints —";
  routeStepEl.innerHTML = STEP_TEXT[routeStep] || "✓ Route updated — see the colored path on the map";
  document.getElementById("badge-src").textContent =
    draftRoute.source != null ? `Start #${draftRoute.source}` : "Start —";
  document.getElementById("badge-obj").textContent = objLabel;
  document.getElementById("badge-tgt").textContent =
    draftRoute.target != null ? `Goal #${draftRoute.target}` : "Goal —";
  const finishBtn = document.getElementById("btn-finish-objectives");
  if (finishBtn) {
    const show = routeStep === "objective" && objs.length >= 1;
    finishBtn.classList.toggle("hidden", !show);
    finishBtn.textContent = `Waypoints done (${objs.length}) — pick goal`;
  }
  updateMapHint();
}

function syncFromServer(snapshot) {
  const dest = snapshot.destinations || {};
  const objs = [...(dest.objectives || [])];
  draftRoute = {
    source: dest.source ?? null,
    objectives: objs,
    target: dest.target ?? null,
  };
  routeStep = "source";
  document.getElementById("src").value = dest.source ?? "";
  document.getElementById("obj").value = objs.join(",");
  document.getElementById("tgt").value = dest.target ?? "";
  updateRouteUI();
}

function renderStats(snapshot) {
  const v = snapshot.verification || {};
  const verifyLine = v.ok
    ? `<div class="verify ok"><b>Verify</b> ✓ ${v.message}</div>`
    : v.message
      ? `<div class="verify fail"><b>Verify</b> ✗ ${v.message}</div>`
      : "";
  const update = snapshot.tree_update;
  const replanText = snapshot.replan_mode === "warm_start" && update
    ? `Warm-start (kept ${update.retained_tree_nodes}/${update.previous_tree_nodes} tree nodes)`
    : snapshot.replan_mode === "race"
      ? "Race (first feasible wins; only better cost replaces)"
    : snapshot.replan_mode === "resume"
      ? "Anytime continue search"
      : snapshot.replan_mode === "ego_reseed"
        ? "Replan from ego pose"
        : "Fresh search";
  const algoLabel = {
    race: "Race orchestrator",
    imomd: "IMOMD-RRT*",
    greedy: "Greedy",
    exact: "Exact (Dijkstra+TSP)",
  }[snapshot.algorithm_id] || (snapshot.algorithm_id || "imomd");
  statsEl.innerHTML = `
    <div><b>Map</b> ${snapshot.map_name} (${snapshot.node_count} nodes)</div>
    <div><b>Solution from</b> ${algoLabel}</div>
    <div><b>Path cost</b> ${snapshot.cost != null ? snapshot.cost.toFixed(1) + " m" : "—"}</div>
    <div><b>Dijkstra oracle</b> ${v.oracle_cost != null ? v.oracle_cost.toFixed(1) + " m" : "—"}</div>
    <div><b>Ego</b> #${snapshot.ego_node ?? "—"}</div>
    <div><b>Anytime</b> ${snapshot.auto_anytime ? "improving" : "paused"}</div>
    <div><b>V2X sim</b> ${snapshot.auto_v2x ? "running" : "stopped"}</div>
    <div><b>Replan mode</b> ${replanText}</div>
    ${verifyLine}
  `;
  eventsEl.innerHTML = (snapshot.events || [])
    .slice()
    .reverse()
    .map((e) => `<li>${e}</li>`)
    .join("");
  drawCostChart(snapshot);
}

function renderAlgoSelect(snapshot) {
  const select = document.getElementById("algo-select");
  if (!select || !snapshot.algorithms) return;
  select.innerHTML = snapshot.algorithms
    .map((algo) => {
      const disabled = algo.available ? "" : "disabled";
      const label = algo.available ? algo.name : `${algo.name} (coming soon)`;
      const selected = algo.id === (snapshot.algorithm_id || "imomd") ? "selected" : "";
      return `<option value="${algo.id}" ${disabled} ${selected}>${label}</option>`;
    })
    .join("");
}

function setLoading(on, text = "Loading map…") {
  const el = document.getElementById("loading");
  const label = document.getElementById("loading-text");
  if (!el) return;
  el.classList.toggle("hidden", !on);
  if (label) label.textContent = text;
}

function resizeMapCanvas() {
  const panel = canvas.parentElement;
  if (!panel) return;
  const rect = panel.getBoundingClientRect();
  const legendH = 36;
  const w = Math.max(640, Math.floor(rect.width - 24));
  const h = Math.max(420, Math.floor(rect.height - legendH - 28));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
    if (state) drawMap(state);
  }
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
  const desc = document.getElementById("map-desc");
  if (desc) {
    const meta = (snapshot.available_maps || []).find((m) => m.key === current);
    desc.textContent = meta?.description || "";
  }
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
  renderAlgoSelect(state);
}

async function postJson(url, body, { loadingText } = {}) {
  if (loadingText) setLoading(true, loadingText);
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: body == null ? "null" : JSON.stringify(body),
    });
    if (!res.ok) {
      const err = await res.text();
      alert(`Request failed: ${err || res.statusText}\n\nTry Smart route`);
      await fetchState();
      return null;
    }
    const data = await res.json();
    await fetchState();
    return data;
  } finally {
    if (loadingText) setLoading(false);
  }
}

async function submitRoute(source, objectives, target) {
  const objs = Array.isArray(objectives) ? objectives : [objectives];
  return postJson(
    `${API}/api/destinations`,
    {
      source,
      objectives: objs,
      target,
    },
    { loadingText: "Planning route…" },
  );
}

async function switchMap(mapKey) {
  const heavy = mapKey === "chicago_mega" || mapKey === "chicago_osm";
  const data = await postJson(
    `${API}/api/map`,
    { map_key: mapKey },
    { loadingText: heavy ? "Loading mega-city map and planning…" : "Switching map…" },
  );
  trafficPolygon = [];
  selectedTrafficEdges = [];
  updateTrafficUI();
  resizeMapCanvas();
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
  const snapRadius = (state?.node_count || 0) > 1200 ? 28 : 56;
  return bestD <= snapRadius ? best : null;
}

canvas.addEventListener("click", async (ev) => {
  if (!state) return;
  const point = canvasPoint(ev);

  if (interactionMode === "route") {
    const nodeId = pickNodeAt(point.x, point.y);
    if (nodeId == null) return;
    if (routeStep === "source") {
      draftRoute = { source: nodeId, objectives: [], target: null };
      routeStep = "objective";
    } else if (routeStep === "objective") {
      if (nodeId === draftRoute.source) {
        alert("A waypoint cannot be the same as the start");
        return;
      }
      if (draftRoute.objectives.includes(nodeId)) {
        // Clicking an existing waypoint removes it (easy undo).
        draftRoute.objectives = draftRoute.objectives.filter((x) => x !== nodeId);
      } else {
        draftRoute.objectives.push(nodeId);
      }
      // Stay on objective step so the user can add more waypoints.
    } else if (routeStep === "target") {
      if (
        nodeId === draftRoute.source ||
        draftRoute.objectives.includes(nodeId)
      ) {
        alert("Goal cannot match the start or a waypoint");
        return;
      }
      draftRoute.target = nodeId;
      routeStep = "done";
      updateRouteUI();
      drawMap(state);
      await submitRoute(draftRoute.source, draftRoute.objectives, draftRoute.target);
      return;
    }
    updateRouteUI();
    drawMap(state);
    return;
  }

  if (interactionMode === "ego") {
    const nodeId = pickNodeAt(point.x, point.y);
    if (nodeId == null) return;
    await postJson(`${API}/api/ego`, { node: nodeId });
    return;
  }

  // traffic mode
  trafficPolygon.push(point);
  recomputeSelectedTrafficEdges();
  drawMap(state);
});

document.getElementById("btn-smart").onclick = () =>
  postJson(`${API}/api/destinations/auto`, null, { loadingText: "Finding a smart route…" });

document.getElementById("btn-reset-route").onclick = () => {
  routeStep = "source";
  draftRoute = { source: null, objectives: [], target: null };
  updateRouteUI();
  if (state) drawMap(state);
};

document.getElementById("btn-finish-objectives").onclick = () => {
  if (routeStep !== "objective" || !(draftRoute.objectives || []).length) {
    alert("Select at least one waypoint first");
    return;
  }
  routeStep = "target";
  updateRouteUI();
  if (state) drawMap(state);
};

document.getElementById("mode-route").onclick = () => setMode("route");
document.getElementById("mode-traffic").onclick = () => setMode("traffic");
document.getElementById("mode-ego").onclick = () => setMode("ego");
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
    alert("Outline at least 3 points to select lanes");
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
document.getElementById("btn-anytime-on").onclick = () => postJson(`${API}/api/anytime?enabled=true`, null);
document.getElementById("btn-anytime-off").onclick = () => postJson(`${API}/api/anytime?enabled=false`, null);

document.getElementById("btn-dest").onclick = () => {
  const source = Number(document.getElementById("src").value);
  const objectives = document
    .getElementById("obj")
    .value.split(/[,\s]+/)
    .map((x) => Number(x.trim()))
    .filter((x) => Number.isFinite(x));
  const target = Number(document.getElementById("tgt").value);
  if (!objectives.length) {
    alert("Enter at least one waypoint (comma-separated allowed)");
    return;
  }
  return submitRoute(source, objectives, target);
};

function connectWs() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/v2x`);
  ws.onmessage = (msg) => {
    const data = JSON.parse(msg.data);
    if (!data.state) return;

    // plan_update sends a lightweight delta (no full OSM view) so we merge.
    if (data.type === "plan_update" && state && state.view) {
      Object.assign(state, data.state);
    } else {
      state = data.state;
    }
    if (data.result) {
      state.path = data.result.path;
      state.cost = data.result.cost;
      if (data.result.visit_order) state.visit_order = data.result.visit_order;
      if (data.result.replan_mode) state.replan_mode = data.result.replan_mode;
    }
    // Keep in-progress route drafting intact for heartbeats / plan_update.
    drawMap(state);
    renderStats(state);
    if (data.type !== "plan_update") {
      renderMapSelect(state);
      renderAlgoSelect(state);
    }
  };
  ws.onclose = () => setTimeout(connectWs, 2000);
}

setMode("route");
window.addEventListener("resize", resizeMapCanvas);
resizeMapCanvas();
fetchState().then(() => {
  resizeMapCanvas();
  connectWs();
});
