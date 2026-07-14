# Realtime Pluggable Navigation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship an event-driven `NavigationSession` with a pluggable planner trait, stream anytime path improvements to the demo UI, and support traffic + ego-position updates under one architecture.

**Architecture:** Thin Rust `src/navigation/` owns events, `PlannerPlugin`, `ImomdPlugin`, and `NavigationSession`. Demo FastAPI becomes a gateway that applies events and broadcasts `PlanUpdate`s over the existing WebSocket. Future LPA*/D* Lite plugins implement the same trait without changing the session or UI contract.

**Tech Stack:** Rust, existing `ImomdRrtStar`, PyO3, FastAPI, Canvas JS

---

## File map

| File | Role |
|---|---|
| `src/navigation/mod.rs` | Module exports |
| `src/navigation/events.rs` | `DomainEvent`, `UpdateReason`, `PlanUpdate` |
| `src/navigation/plugin.rs` | `PlannerPlugin` trait |
| `src/navigation/imomd_plugin.rs` | Adapter over `ImomdRrtStar` |
| `src/navigation/session.rs` | `NavigationSession` |
| `tests/navigation_session.rs` | Session/plugin integration tests |
| `src/lib.rs` / `src/python/mod.rs` | Export + Python bindings |
| `demo/app/main.py` | Use session; stream updates |
| `demo/static/app.js` / `index.html` | Anytime, ego, cost curve |
| `docs/navigation-plugins.md` | How to add a plugin |

---

### Task 1: Core types + PlannerPlugin + ImomdPlugin

**Files:**
- Create: `src/navigation/{mod,events,plugin,imomd_plugin}.rs`
- Modify: `src/lib.rs`
- Test: `tests/navigation_session.rs` (plugin reset/step first)

- [ ] **Step 1:** Add failing test that `ImomdPlugin` resets on fake map and `continue_search` returns a path within budget.
- [ ] **Step 2:** Implement `events`, `plugin`, `imomd_plugin`; wire `pub mod navigation`.
- [ ] **Step 3:** `cargo test --test navigation_session` green for plugin-only cases.

### Task 2: NavigationSession (traffic + ego + continue)

**Files:**
- Create: `src/navigation/session.rs`
- Modify: `tests/navigation_session.rs`

- [ ] **Step 1:** Failing tests for `TrafficChanged` warm-start reason and `EgoMoved` reseeding source.
- [ ] **Step 2:** Implement session event dispatch + sequence numbers.
- [ ] **Step 3:** Tests green.

### Task 3: Python bindings

**Files:**
- Modify: `src/python/mod.rs`, `python/IMOMD_RRTStar/__init__.py`, `python/IMOMD_RRTStar/__init__.pyi`
- Test: `test/test_navigation_session.py`

- [ ] **Step 1:** Expose `NavigationSession` / `PlanUpdate` dicts.
- [ ] **Step 2:** unittest for traffic + continue_search streaming.

### Task 4: Demo streaming gateway

**Files:**
- Modify: `demo/app/main.py`
- Test: extend `scripts/test_demo_api.py` or `demo/tests/`

- [ ] **Step 1:** Session-backed replan; WS `plan_update` messages during anytime slices.
- [ ] **Step 2:** Endpoints for ego set + anytime enable.
- [ ] **Step 3:** API smoke still passes; new stream assertions.

### Task 5: Frontend anytime + ego + cost curve

**Files:**
- Modify: `demo/static/app.js`, `demo/static/index.html`

- [ ] **Step 1:** Handle `plan_update`; morph path; append cost history.
- [ ] **Step 2:** Ego marker + set-ego click mode; anytime toggle.
- [ ] **Step 3:** Manual browser check against user stories.

### Task 6: Plugin docs + placeholder registry

**Files:**
- Create: `docs/navigation-plugins.md`
- Modify: demo algorithm selector (imomd live; others disabled)

- [ ] **Step 1:** Document trait + registration steps.
- [ ] **Step 2:** UI selector stub for future algorithms.
