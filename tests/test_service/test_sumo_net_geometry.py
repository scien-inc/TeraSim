import math
from pathlib import Path
import xml.etree.ElementTree as ET

from terasim_service.utils.sumo_net_geometry import (
    find_geometry_discontinuities,
    parse_shape,
)
from terasim_service.utils.sumo_lane_geometry import (
    select_route_aware_lane_projection,
)
from scripts.repair_odaiba_sumo_net import (
    NODE_205_CONNECTION_LANELETS,
    NODE_205_INCOMING_LANELETS,
    NODE_205_OUTGOING_LANELETS,
    NODE_825_INVALID_CONNECTIONS,
    NODE_825_VALID_CONNECTION,
    SOURCE_LANELET_IDS,
    _load_lanelet_centerlines,
    _normal_connection_keys,
    _restore_reversed_short_internal_lanes,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
MAP_DIR = REPO_ROOT / "examples/maps/odaiba_ll2/tlmappings_0708"
ORIGINAL_NET = MAP_DIR / "network.net.xml"
FIXED_NET = MAP_DIR / "network.fixed_geometry.net.xml"
REPAIRED_NET = MAP_DIR / "network.repaired_geometry.net.xml"
SOURCE_OSM = REPO_ROOT / "examples/maps/odaiba_ll2/odaiba_ll2_raw.osm"
ACTIVE_ROUTES = (
    MAP_DIR / "period_0p2_filter_check/vehicles.filtered_r300.rou.xml"
)



def _issues_by_object(net_file):
    issues = find_geometry_discontinuities(net_file, max_angle_degrees=45.0)
    return issues, {issue.object_id for issue in issues}


def _lane(net_file, lane_id):
    root = ET.parse(net_file).getroot()
    return next(lane for lane in root.iter("lane") if lane.get("id") == lane_id)


def _direct_short_source_via_candidates():
    """Return direct 0.25 m via lanes whose source shape owns the movement."""
    repaired_root = ET.parse(REPAIRED_NET).getroot()
    source_root = ET.parse(FIXED_NET).getroot()
    repaired_lanes = {
        lane.get("id"): lane for lane in repaired_root.iter("lane") if lane.get("id")
    }
    source_lanes = {
        lane.get("id"): lane for lane in source_root.iter("lane") if lane.get("id")
    }
    source_connections = {
        (
            connection.get("from"),
            connection.get("fromLane"),
            connection.get("to"),
            connection.get("toLane"),
        ): connection
        for connection in source_root.iter("connection")
    }
    repaired_connections = list(repaired_root.iter("connection"))

    candidates = []
    for connection in repaired_connections:
        via = connection.get("via")
        from_edge = connection.get("from", "")
        to_edge = connection.get("to", "")
        if not via or from_edge.startswith(":") or to_edge.startswith(":"):
            continue
        try:
            internal_edge, internal_lane_index = via.rsplit("_", 1)
        except ValueError:
            continue
        downstream = [
            candidate
            for candidate in repaired_connections
            if candidate.get("from") == internal_edge
            and candidate.get("fromLane") == internal_lane_index
            and candidate.get("to") == to_edge
            and candidate.get("toLane") == connection.get("toLane")
        ]
        if len(downstream) != 1 or downstream[0].get("via"):
            # A chained via is a legitimate split-junction movement, not a
            # short direct connector owned by this connection.
            continue

        repaired_lane = repaired_lanes.get(via)
        source_lane = source_lanes.get(via)
        source_connection = source_connections.get(
            (
                from_edge,
                connection.get("fromLane"),
                to_edge,
                connection.get("toLane"),
            )
        )
        if repaired_lane is None or source_lane is None or source_connection is None:
            continue
        guarded_elements = (
            connection,
            repaired_lane,
            source_lane,
            source_connection,
        )
        lengths = tuple(element.get("length") for element in guarded_elements)
        if any(element.get("shape") is None for element in guarded_elements):
            continue
        connection_shape = parse_shape(connection.get("shape"))
        repaired_shape = parse_shape(repaired_lane.get("shape"))
        source_shape = parse_shape(source_lane.get("shape"))
        source_connection_shape = parse_shape(source_connection.get("shape"))
        guarded_shapes = (
            connection_shape, repaired_shape, source_shape, source_connection_shape
        )
        if not all(len(shape) == 2 for shape in guarded_shapes):
            continue
        if not all(abs(float(length) - 0.250) <= 1e-9 for length in lengths):
            continue
        if source_connection.get("via") != via:
            continue
        if source_connection_shape != connection_shape:
            continue
        if source_shape != connection_shape:
            continue
        if sum(
            (repaired_shape[0][index] - source_shape[0][index]) ** 2 for index in (0, 1)
        ) > 4e-6:
            continue
        candidates.append((via, connection_shape, repaired_shape))
    return candidates


def _is_reversed_or_degenerate(source_shape, rebuilt_shape):
    source_dx = source_shape[1][0] - source_shape[0][0]
    source_dy = source_shape[1][1] - source_shape[0][1]
    rebuilt_dx = rebuilt_shape[1][0] - rebuilt_shape[0][0]
    rebuilt_dy = rebuilt_shape[1][1] - rebuilt_shape[0][1]
    rebuilt_length_sq = rebuilt_dx * rebuilt_dx + rebuilt_dy * rebuilt_dy
    if rebuilt_length_sq <= 1e-18:
        return True

    source_length_sq = source_dx * source_dx + source_dy * source_dy
    return (
        source_length_sq > 1e-18
        and source_dx * rebuilt_dx + source_dy * rebuilt_dy < 0.0
    )


def _synthetic_short_internal_net(direct_shape, zero_shape, split_shape):
    return ET.fromstring(
        f"""
        <net>
            <edge id=":direct_0" function="internal">
                <lane id=":direct_0_0" index="0" length="0.250" shape="{direct_shape}" />
            </edge>
            <edge id=":zero_0" function="internal">
                <lane id=":zero_0_0" index="0" length="0.250" shape="{zero_shape}" />
            </edge>
            <edge id=":split_0" function="internal">
                <lane id=":split_0_0" index="0" length="0.250" shape="{split_shape}" />
            </edge>
            <connection from="in" fromLane="0" to="out" toLane="0"
                length="0.250" shape="0,0,0 0.25,0,0" via=":direct_0_0" />
            <connection from=":direct_0" fromLane="0" to="out" toLane="0" />
            <connection from="in" fromLane="1" to="out" toLane="1"
                length="0.250" shape="1,0,0 1.25,0,0" via=":zero_0_0" />
            <connection from=":zero_0" fromLane="0" to="out" toLane="1" />
            <connection from="in" fromLane="2" to="out" toLane="2"
                length="0.250" shape="2,0,0 2.25,0,0" via=":split_0_0" />
            <connection from=":split_0" fromLane="0" to="out" toLane="2"
                via=":split_1_0" />
        </net>
        """
    )


def test_short_internal_repair_restores_only_direct_source_backed_defects():
    source_root = _synthetic_short_internal_net(
        direct_shape="0,0,0 0.25,0,0",
        zero_shape="1,0,0 1.25,0,0",
        split_shape="2,0,0 2.25,0,0",
    )
    rebuilt_root = _synthetic_short_internal_net(
        direct_shape="0,0,0 -0.25,0,0",
        zero_shape="1,0,0 1,0,0",
        split_shape="2,0,0 1.75,0,0",
    )
    lanes = {lane.get("id"): lane for lane in rebuilt_root.iter("lane")}
    split_before = lanes[":split_0_0"].get("shape")

    assert _restore_reversed_short_internal_lanes(rebuilt_root, source_root) == 2
    assert lanes[":direct_0_0"].get("shape") == "0,0,0 0.25,0,0"
    assert lanes[":zero_0_0"].get("shape") == "1,0,0 1.25,0,0"
    assert lanes[":split_0_0"].get("shape") == split_before


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


def test_repaired_short_direct_via_lanes_keep_lanelet2_source_geometry():
    # These source lanes come from the fixed net generated from odaiba_ll2_raw.osm.
    candidates = _direct_short_source_via_candidates()
    candidate_lane_ids = {lane_id for lane_id, _, _ in candidates}

    assert candidates
    assert ":node_1046_0_1" in candidate_lane_ids
    assert [
        lane_id
        for lane_id, source_shape, repaired_shape in candidates
        if _is_reversed_or_degenerate(source_shape, repaired_shape)
    ] == []


def test_repaired_node_1046_lane_keeps_lanelet2_source_geometry():
    source_lane = _lane(FIXED_NET, ":node_1046_0_1")
    repaired_lane = _lane(REPAIRED_NET, ":node_1046_0_1")
    expected_shape = (
        (89361.382, 42903.444, 6.939),
        (89361.464, 42903.208, 6.939),
    )

    assert parse_shape(source_lane.get("shape")) == expected_shape
    assert parse_shape(repaired_lane.get("shape")) == expected_shape
    assert float(repaired_lane.get("length")) == 0.250


def test_repaired_node_205_follows_lanelet2_source_chains():
    centerlines = _load_lanelet_centerlines(SOURCE_OSM)

    for lane_id, source_name in NODE_205_INCOMING_LANELETS.items():
        actual = parse_shape(_lane(REPAIRED_NET, lane_id).get("shape"))
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        assert actual[-1] == tuple(round(value, 3) for value in source[-1])

    for lane_id, source_name in NODE_205_OUTGOING_LANELETS.items():
        actual = parse_shape(_lane(REPAIRED_NET, lane_id).get("shape"))
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        assert actual[0] == tuple(round(value, 3) for value in source[0])

    root = ET.parse(REPAIRED_NET).getroot()
    connections = {
        (
            connection.get("from"),
            connection.get("fromLane"),
            connection.get("to"),
            connection.get("toLane"),
        ): connection
        for connection in root.findall("connection")
    }
    for key, (source_name, expected_via) in NODE_205_CONNECTION_LANELETS.items():
        source = tuple(
            tuple(round(value, 3) for value in point)
            for point in centerlines[SOURCE_LANELET_IDS[source_name]]
        )
        connection = connections[key]
        assert connection.get("via") == expected_via
        assert parse_shape(connection.get("shape")) == source
        assert parse_shape(_lane(REPAIRED_NET, expected_via).get("shape")) == source


def test_repaired_node_825_removes_only_connections_rejected_by_lanelet2_topology():
    centerlines = _load_lanelet_centerlines(SOURCE_OSM)
    incoming_end = centerlines[SOURCE_LANELET_IDS["node_825_incoming"]][-1]
    valid_start = centerlines[SOURCE_LANELET_IDS["node_825_valid_outgoing"]][0]
    invalid_starts = (
        centerlines[SOURCE_LANELET_IDS["node_825_invalid_outgoing_0"]][0],
        centerlines[SOURCE_LANELET_IDS["node_825_invalid_outgoing_1"]][0],
    )

    assert math.dist(incoming_end[:2], valid_start[:2]) < 1e-6
    assert all(math.dist(incoming_end[:2], point[:2]) > 8.0 for point in invalid_starts)

    source_connections = _normal_connection_keys(FIXED_NET)
    repaired_connections = _normal_connection_keys(REPAIRED_NET)
    assert source_connections - repaired_connections == NODE_825_INVALID_CONNECTIONS
    assert repaired_connections - source_connections == set()
    assert NODE_825_VALID_CONNECTION in repaired_connections

def test_active_odaiba_routes_follow_repaired_normal_edge_topology():
    normal_edge_connections = {
        (from_edge, to_edge)
        for from_edge, _, to_edge, _ in _normal_connection_keys(REPAIRED_NET)
    }
    root = ET.parse(ACTIVE_ROUTES).getroot()
    routes = [
        (f"route:{route.get('id')}", route.get("edges", "").split())
        for route in root.findall("route")
    ]
    routes.extend(
        (
            f"vehicle:{vehicle.get('id')}",
            vehicle.find("route").get("edges", "").split(),
        )
        for vehicle in root.findall("vehicle")
    )
    invalid_transitions = [
        (owner, from_edge, to_edge)
        for owner, edges in routes
        for from_edge, to_edge in zip(edges, edges[1:])
        if (from_edge, to_edge) not in normal_edge_connections
    ]

    assert invalid_transitions == []

def test_edge_1384_field_pose_projects_to_lanelet2_internal_connector():
    lane = _lane(REPAIRED_NET, ":node_205_4_0")
    shape = parse_shape(lane.get("shape"))
    candidate = {
        "lane_id": lane.get("id"),
        "edge_id": ":node_205_4",
        "lane_index": 0,
        "shape": shape,
        "shape3d": shape,
        "length": float(lane.get("length")),
    }

    projection = select_route_aware_lane_projection(
        position=(89641.13103695, 42232.28135523),
        position_z=6.32,
        sumo_angle=326.55926422121814,
        lane_candidates=[candidate],
        current_lane_id=lane.get("id"),
        max_distance=8.0,
        max_elevation_error=2.0,
        max_heading_error=90.0,
        prefer_current_lane=True,
    )

    assert projection is not None
    assert projection["lane_id"] == ":node_205_4_0"
    assert abs(projection["distance"] - 2.1115860462) < 1e-6
    assert abs(projection["heading_error"]) < 1e-6
    assert abs(projection["elevation_error"] - 0.0143014505) < 1e-6


def test_node_1046_field_pose_projects_to_authoritative_current_lane():
    lane = _lane(REPAIRED_NET, ":node_1046_0_1")
    shape = parse_shape(lane.get("shape"))
    candidate = {
        "lane_id": lane.get("id"),
        "edge_id": ":node_1046_0",
        "lane_index": 1,
        "shape": shape,
        "shape3d": shape,
        "length": float(lane.get("length")),
    }
    projection = select_route_aware_lane_projection(
        position=(89359.87321072252, 42895.57717352966),
        position_z=7.010738372802734,
        sumo_angle=165.9237289428711,
        lane_candidates=[candidate],
        current_lane_id=lane.get("id"),
        max_distance=8.0,
        max_elevation_error=2.0,
        max_heading_error=90.0,
        prefer_current_lane=True,
    )

    assert projection is not None
    assert projection["lane_id"] == ":node_1046_0_1"
    assert abs(projection["distance"] - 7.7948780071) < 1e-6
    assert projection["lane_position"] == 0.250
    assert projection["heading_error"] < 6.0

    assert (
        select_route_aware_lane_projection(
            position=(89450.0, 43000.0),
            position_z=6.939,
            sumo_angle=165.9237289428711,
            lane_candidates=[candidate],
            current_lane_id=lane.get("id"),
            max_distance=8.0,
            max_elevation_error=2.0,
            max_heading_error=90.0,
            prefer_current_lane=True,
        )
        is None
    )
