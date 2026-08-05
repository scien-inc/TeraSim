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
