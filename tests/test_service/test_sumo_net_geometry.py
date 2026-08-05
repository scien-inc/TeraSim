from pathlib import Path
import xml.etree.ElementTree as ET

from terasim_service.utils.sumo_net_geometry import (
    find_geometry_discontinuities,
    parse_shape,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
MAP_DIR = REPO_ROOT / "examples/maps/odaiba_ll2/tlmappings_0708"
ORIGINAL_NET = MAP_DIR / "network.net.xml"
FIXED_NET = MAP_DIR / "network.fixed_geometry.net.xml"
REPAIRED_NET = MAP_DIR / "network.repaired_geometry.net.xml"


def _issues_by_object(net_file):
    issues = find_geometry_discontinuities(net_file, max_angle_degrees=45.0)
    return issues, {issue.object_id for issue in issues}


def _lane(net_file, lane_id):
    root = ET.parse(net_file).getroot()
    return next(lane for lane in root.iter("lane") if lane.get("id") == lane_id)


def test_fixed_odaiba_geometry_removes_node_119_discontinuities():
    original_issues, original_objects = _issues_by_object(ORIGINAL_NET)
    fixed_issues, fixed_objects = _issues_by_object(FIXED_NET)

    assert ":node_119_5_0" in original_objects
    assert "edge_1237_1->edge_1243_0" in original_objects
    assert ":node_119_5_0->edge_1243_0" in original_objects
    assert ":node_119_5_0" not in fixed_objects

    target_connections = {
        "edge_1237_1->edge_1243_0",
        ":node_119_5_0->edge_1243_0",
        "edge_1231_0->edge_1243_0",
        ":node_119_0_0->edge_1243_0",
    }
    assert not {
        issue.object_id
        for issue in fixed_issues
        if issue.object_id in target_connections
    }
    assert len(fixed_issues) < len(original_issues)


def test_fixed_odaiba_geometry_introduces_no_new_45_degree_discontinuities():
    original_issues = find_geometry_discontinuities(
        ORIGINAL_NET, max_angle_degrees=45.0
    )
    fixed_issues = find_geometry_discontinuities(FIXED_NET, max_angle_degrees=45.0)

    original_keys = {issue.key for issue in original_issues}
    new_issues = [issue for issue in fixed_issues if issue.key not in original_keys]
    assert new_issues == []


def test_fixed_node_119_shapes_keep_lanelet2_source_endpoints():
    turn_lane = _lane(FIXED_NET, ":node_119_5_0")
    outgoing_lane = _lane(FIXED_NET, "edge_1243_0")
    turn_shape = parse_shape(turn_lane.get("shape"))
    outgoing_shape = parse_shape(outgoing_lane.get("shape"))

    # odaiba_ll2_raw.osm lanelets 2224178 -> 2224181 share this centerline point.
    expected_transition = (89787.964, 42456.978, 3.390)
    assert turn_shape[-1] == expected_transition
    assert outgoing_shape[0] == expected_transition

    assert float(turn_lane.get("length")) == 28.675
    assert float(outgoing_lane.get("length")) == 28.727


def test_original_odaiba_net_is_preserved():
    original_outgoing = _lane(ORIGINAL_NET, "edge_1243_0")
    fixed_outgoing = _lane(FIXED_NET, "edge_1243_0")

    assert parse_shape(original_outgoing.get("shape"))[0] == (
        89800.337,
        42437.983,
        3.335,
    )
    assert parse_shape(fixed_outgoing.get("shape"))[0] == (
        89787.964,
        42456.978,
        3.390,
    )


def test_repaired_odaiba_geometry_removes_connection_discontinuities():
    fixed_issues = find_geometry_discontinuities(
        FIXED_NET, max_angle_degrees=45.0
    )
    repaired_issues = find_geometry_discontinuities(
        REPAIRED_NET, max_angle_degrees=45.0
    )

    assert len(repaired_issues) < len(fixed_issues)
    assert not {
        issue.kind
        for issue in repaired_issues
        if issue.kind in {"connection_shape", "connection_boundary"}
    }
    fixed_keys = {issue.key for issue in fixed_issues}
    assert len(repaired_issues) == 7
    assert not [issue for issue in repaired_issues if issue.key not in fixed_keys]
    assert max(issue.angle_degrees for issue in repaired_issues) < 90.0

    repaired_objects = {issue.object_id for issue in repaired_issues}
    assert "edge_1231_1->edge_1235_0" not in repaired_objects
    assert "edge_531_0" not in repaired_objects
    assert "edge_531_0->edge_95_0" not in repaired_objects


def test_repaired_node_119_follows_lanelet2_source_chain():
    incoming = _lane(REPAIRED_NET, "edge_1231_1")
    turn = _lane(REPAIRED_NET, ":node_119_1_0")
    outgoing = _lane(REPAIRED_NET, "edge_1235_0")
    incoming_shape = parse_shape(incoming.get("shape"))
    turn_shape = parse_shape(turn.get("shape"))
    outgoing_shape = parse_shape(outgoing.get("shape"))

    assert incoming_shape[-1] == (89810.506, 42416.284, 3.404)
    assert turn_shape[0] == incoming_shape[-1]
    assert turn_shape[-1] == (89784.935, 42425.603, 3.433)
    assert outgoing_shape[0] == (89784.725, 42425.467, 3.435)

    assert float(incoming.get("length")) == 59.701
    assert float(turn.get("length")) == 31.606
    assert float(outgoing.get("length")) == 112.981


def test_repaired_edge_531_has_continuous_short_connector():
    incoming_shape = parse_shape(_lane(REPAIRED_NET, "edge_531_0").get("shape"))
    connector = _lane(REPAIRED_NET, ":ia_2017578_3_0")
    connector_shape = parse_shape(connector.get("shape"))
    outgoing_shape = parse_shape(_lane(REPAIRED_NET, "edge_95_0").get("shape"))

    assert incoming_shape[-1] == (90102.623, 43800.699, 3.853)
    assert connector_shape[0] == incoming_shape[-1]
    assert connector_shape[-1] == outgoing_shape[0]
    assert float(connector.get("length")) == 0.250


def test_previous_fixed_odaiba_net_is_preserved():
    fixed_incoming = _lane(FIXED_NET, "edge_1231_1")
    repaired_incoming = _lane(REPAIRED_NET, "edge_1231_1")

    assert parse_shape(fixed_incoming.get("shape"))[-1] == (
        89798.569,
        42434.772,
        3.418,
    )
    assert parse_shape(repaired_incoming.get("shape"))[-1] == (
        89810.506,
        42416.284,
        3.404,
    )
