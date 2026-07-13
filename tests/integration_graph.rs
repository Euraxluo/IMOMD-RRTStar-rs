use IMOMD_RRTStar::graph::RoadGraph;
use IMOMD_RRTStar::map::{FakeMapLoader, MapLoader};

#[test]
fn integration_fake_map_1_loads() {
    let g = FakeMapLoader::new(-1).load().expect("load fake map 1");
    assert_eq!(g.node_count(), 4);
    assert!(g.edge_weight(0, 1).is_some());
}

#[test]
fn integration_fake_map_2_connected() {
    let g = FakeMapLoader::new(-2).load().expect("load fake map 2");
    assert_eq!(g.node_count(), 7);
    // node 0 and node 6 should be in the same connected component via edges
    assert!(g.neighbors(0).len() >= 1);
    assert!(g.neighbors(6).len() >= 1);
}
