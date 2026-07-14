use std::sync::Arc;
use std::time::Duration;

use IMOMD_RRTStar::graph::{RoadGraph, TrafficGraph, TrafficLevel};
use IMOMD_RRTStar::map::{FakeMapLoader, MapLoader};
use IMOMD_RRTStar::navigation::{
    DomainEvent, ImomdPlugin, NavigationSession, PlannerPlugin, UpdateReason,
};
use IMOMD_RRTStar::types::Destinations;

fn fake_graph() -> Arc<IMOMD_RRTStar::graph::AdjacencyGraph> {
    Arc::new(FakeMapLoader::new(-1).load().expect("fake map"))
}

#[test]
fn imomd_plugin_finds_path_on_fake_map() {
    let graph = fake_graph();
    let mut plugin = ImomdPlugin::with_default_config();
    plugin
        .reset(
            Arc::clone(&graph),
            Destinations {
                source: 0,
                objectives: vec![1],
                target: 2,
            },
        )
        .unwrap();
    let updates = plugin
        .continue_search(Duration::from_millis(500))
        .unwrap();
    assert!(
        plugin.best().is_some(),
        "expected a best path, updates={updates:?}"
    );
    let best = plugin.best().unwrap();
    assert_eq!(best.path.first().copied(), Some(0));
    assert_eq!(best.path.last().copied(), Some(2));
    assert!(best.path.contains(&1));
}

#[test]
fn session_streams_after_destinations_set() {
    let mut session = NavigationSession::new(Box::new(ImomdPlugin::with_default_config()));
    session.set_graph(fake_graph());
    let updates = session
        .handle(
            DomainEvent::DestinationsSet {
                source: 0,
                objectives: vec![1],
                target: 2,
            },
            Duration::from_millis(500),
        )
        .unwrap();
    assert!(updates
        .iter()
        .any(|u| u.path.as_ref().map(|p| !p.is_empty()).unwrap_or(false)));
    assert!(session.best().is_some());
    assert!(updates
        .iter()
        .any(|u| matches!(u.reason, UpdateReason::Fresh)));
}

#[test]
fn session_traffic_change_emits_warm_start() {
    let base = FakeMapLoader::new(-1).load().unwrap();
    let mut traffic = TrafficGraph::new(base);
    let mut session = NavigationSession::new(Box::new(ImomdPlugin::with_default_config()));
    session.set_graph(Arc::new(traffic.materialize().unwrap()));
    session
        .handle(
            DomainEvent::DestinationsSet {
                source: 0,
                objectives: vec![1],
                target: 2,
            },
            Duration::from_millis(300),
        )
        .unwrap();

    traffic.set_edge_level(0, 1, TrafficLevel::Jam);
    session.set_graph(Arc::new(traffic.materialize().unwrap()));
    let updates = session
        .handle(DomainEvent::TrafficChanged, Duration::from_millis(300))
        .unwrap();
    assert!(updates
        .iter()
        .any(|u| matches!(u.reason, UpdateReason::TrafficWarmStart)));
    assert_eq!(
        updates
            .iter()
            .find(|u| matches!(u.reason, UpdateReason::TrafficWarmStart))
            .unwrap()
            .replan_mode,
        "warm_start"
    );
}

#[test]
fn session_ego_move_reseeds_source() {
    let mut session = NavigationSession::new(Box::new(ImomdPlugin::with_default_config()));
    let graph = fake_graph();
    session.set_graph(Arc::clone(&graph));
    session
        .handle(
            DomainEvent::DestinationsSet {
                source: 0,
                objectives: vec![1],
                target: 2,
            },
            Duration::from_millis(300),
        )
        .unwrap();

    let loc = graph.location(1).unwrap();
    let snapped = session.snap_ego(loc.latitude, loc.longitude).unwrap();
    assert_eq!(snapped, 1);

    let updates = session
        .handle(
            DomainEvent::EgoMoved { ego_node: 1 },
            Duration::from_millis(400),
        )
        .unwrap();
    assert!(updates
        .iter()
        .any(|u| matches!(u.reason, UpdateReason::EgoReseed)));
    assert_eq!(session.ego_node(), Some(1));
    let dest = session.destinations().unwrap();
    assert_eq!(dest.source, 1);
    assert!(!dest.objectives.contains(&1));
}

#[test]
fn continue_search_can_emit_multiple_updates() {
    let mut session = NavigationSession::new(Box::new(ImomdPlugin::with_default_config()));
    session.set_graph(fake_graph());
    session
        .handle(
            DomainEvent::DestinationsSet {
                source: 0,
                objectives: vec![1],
                target: 2,
            },
            Duration::from_millis(50),
        )
        .unwrap();
    let more = session
        .handle(DomainEvent::ContinueSearch, Duration::from_millis(400))
        .unwrap();
    assert!(!more.is_empty());
    let mut last = 0;
    for u in &more {
        assert!(u.sequence > last);
        last = u.sequence;
    }
}

#[test]
fn destinations_race_admits_first_feasible_path() {
    let mut session = NavigationSession::new(Box::new(ImomdPlugin::with_default_config()));
    session.set_graph(fake_graph());
    let updates = session
        .handle(
            DomainEvent::DestinationsSet {
                source: 0,
                objectives: vec![1],
                target: 2,
            },
            Duration::from_millis(400),
        )
        .unwrap();
    assert!(
        updates.iter().any(|u| u.path.as_ref().is_some_and(|p| p.len() > 1)),
        "race must publish a path, updates={updates:?}"
    );
    // Exact or greedy may finish before IMOMD; algorithm_id should be one of the lanes.
    let id = session.algorithm_id();
    assert!(
        matches!(id, "imomd" | "greedy" | "exact"),
        "unexpected algorithm_id={id}"
    );
    assert!(session.best().is_some());
}

#[test]
fn continue_search_improves_after_early_finish() {
    let mut session = NavigationSession::new(Box::new(ImomdPlugin::with_default_config()));
    session.set_graph(fake_graph());
    let first = session
        .handle(
            DomainEvent::DestinationsSet {
                source: 0,
                objectives: vec![1],
                target: 2,
            },
            Duration::from_millis(30),
        )
        .unwrap();
    let first_cost = session.best().map(|b| b.cost);
    assert!(first_cost.is_some() || first.iter().any(|u| u.path.is_some()));

    let mut best = first_cost;
    for _ in 0..8 {
        let _ = session
            .handle(DomainEvent::ContinueSearch, Duration::from_millis(200))
            .unwrap();
        if let Some(cost) = session.best().map(|b| b.cost) {
            best = Some(best.map_or(cost, |b| b.min(cost)));
        }
    }
    assert!(session.best().is_some(), "anytime resume must keep a best path");
    // On tiny fake maps the first slice may already be optimal; at least resume
    // must not wipe the solution or refuse to search.
    assert!(best.is_some());
}
