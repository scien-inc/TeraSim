#!/usr/bin/env python3
"""Rebuild the Odaiba SUMO net without the generated geometry discontinuities.

The input net is never modified.  The script exports it to SUMO Plain XML,
restores source-backed lane geometry, removes discontinuous connection shapes,
removes two node_825 connections rejected by the Lanelet2 topology,
and lets the same netconvert version rebuild junction-internal geometry.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


Point = tuple[float, float, float]
ConnectionKey = tuple[str, str, str, str]

SOURCE_LANELET_IDS = {
    "incoming_left_turn": "2224169",
    "left_turn_connector": "2224170",
    "repeated_point_lane": "2118445",
    "node_205_incoming_0": "2224361",
    "node_205_incoming_1": "2224360",
    "node_205_incoming_2": "2224359",
    "node_205_to_1363_0": "2224364",
    "node_205_to_1368_0": "2224363",
    "node_205_to_1368_1": "2224388",
    "node_205_to_1369_0": "2224387",
    "node_205_to_1368_2": "2224362",
    "node_205_outgoing_1363_0": "2224365",
    "node_205_outgoing_1368_0": "2224374",
    "node_205_outgoing_1368_1": "2224381",
    "node_205_outgoing_1368_2": "2224380",
    "node_205_outgoing_1369_0": "2224375",
    "node_825_incoming": "178481",
    "node_825_valid_outgoing": "1910618",
    "node_825_invalid_outgoing_0": "2306272",
    "node_825_invalid_outgoing_1": "2307310",
}

NODE_205_INCOMING_LANELETS = {
    "edge_1384_0": "node_205_incoming_0",
    "edge_1384_1": "node_205_incoming_1",
    "edge_1384_2": "node_205_incoming_2",
}
NODE_205_OUTGOING_LANELETS = {
    "edge_1363_0": "node_205_outgoing_1363_0",
    "edge_1368_0": "node_205_outgoing_1368_0",
    "edge_1368_1": "node_205_outgoing_1368_1",
    "edge_1368_2": "node_205_outgoing_1368_2",
    "edge_1369_0": "node_205_outgoing_1369_0",
}
NODE_205_CONNECTION_LANELETS: dict[ConnectionKey, tuple[str, str]] = {
    ("edge_1384", "0", "edge_1363", "0"): ("node_205_to_1363_0", ":node_205_0_0"),
    ("edge_1384", "1", "edge_1368", "0"): ("node_205_to_1368_0", ":node_205_1_0"),
    ("edge_1384", "1", "edge_1368", "1"): ("node_205_to_1368_1", ":node_205_1_1"),
    ("edge_1384", "1", "edge_1369", "0"): ("node_205_to_1369_0", ":node_205_3_0"),
    ("edge_1384", "2", "edge_1368", "2"): ("node_205_to_1368_2", ":node_205_4_0"),
}

# These shapes were already corrected from lanelets 2224165, 2224178 and
# 2224181.  Keep them instead of replacing them with a generic junction curve.
PRESERVED_CONNECTIONS: set[ConnectionKey] = {
    ("edge_1231", "0", "edge_1243", "0"),
    ("edge_1237", "1", "edge_1243", "0"),
}

TARGET_CONNECTION: ConnectionKey = ("edge_1231", "1", "edge_1235", "0")

NODE_825_INVALID_CONNECTIONS: set[ConnectionKey] = {
    ("edge_482", "0", "edge_3249", "0"),
    ("edge_482", "0", "edge_3249", "1"),
}
NODE_825_VALID_CONNECTION: ConnectionKey = ("edge_482", "0", "edge_506", "0")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-net", required=True, type=Path)
    parser.add_argument("--source-osm", required=True, type=Path)
    parser.add_argument("--output-net", required=True, type=Path)
    parser.add_argument(
        "--netconvert",
        default="netconvert",
        help="netconvert executable (default: netconvert)",
    )
    return parser.parse_args()


def _point_distance(first: Point, second: Point) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


def _polyline_length(points: list[Point]) -> float:
    return sum(_point_distance(first, second) for first, second in zip(points, points[1:]))


def _format_shape(points: list[Point]) -> str:
    return " ".join(f"{x:.3f},{y:.3f},{z:.3f}" for x, y, z in points)


def _parse_shape(shape: str | None) -> list[Point]:
    if not shape:
        return []
    points = []
    for encoded_point in shape.split():
        values = [float(value) for value in encoded_point.split(",")]
        values.extend([0.0] * (3 - len(values)))
        points.append((values[0], values[1], values[2]))
    return points


def _replace_shape_suffix(
    points: list[Point], replacement: list[Point], max_distance: float = 0.002
) -> list[Point]:
    """Replace the generated tail beginning at an authoritative source point."""

    if len(points) < 2 or len(replacement) < 2:
        raise ValueError("source-backed lane shapes need at least two points")
    match_index = min(
        range(len(points)),
        key=lambda index: _point_distance(points[index], replacement[0]),
    )
    distance = _point_distance(points[match_index], replacement[0])
    if distance > max_distance:
        raise ValueError(
            f"source geometry start is {distance:.3f} m from the generated lane"
        )
    return [*points[:match_index], *replacement]


def _replace_shape_prefix(
    points: list[Point], replacement: list[Point], max_distance: float = 1.0
) -> list[Point]:
    """Replace a generated head while retaining the remaining edge geometry."""

    if len(points) < 2 or len(replacement) < 2:
        raise ValueError("source-backed lane shapes need at least two points")
    match_index = min(
        range(len(points)),
        key=lambda index: _point_distance(points[index], replacement[-1]),
    )
    distance = _point_distance(points[match_index], replacement[-1])
    if distance > max_distance:
        raise ValueError(
            f"source geometry end is {distance:.3f} m from the generated lane"
        )
    return [*replacement, *points[match_index + 1 :]]


def _interpolate(first: Point, second: Point, fraction: float) -> Point:
    return tuple(
        first[index] + (second[index] - first[index]) * fraction for index in range(3)
    )  # type: ignore[return-value]


def _resample(points: list[Point], count: int) -> list[Point]:
    """Resample a boundary by normalized 2D arclength."""

    if count < 2 or len(points) < 2:
        raise ValueError("a lanelet boundary needs at least two points")
    lengths = [0.0]
    for first, second in zip(points, points[1:]):
        lengths.append(lengths[-1] + _point_distance(first, second))
    if lengths[-1] <= 1e-9:
        raise ValueError("a lanelet boundary has zero length")

    result = []
    segment = 0
    for index in range(count):
        target = lengths[-1] * index / (count - 1)
        while segment + 1 < len(lengths) - 1 and lengths[segment + 1] < target:
            segment += 1
        segment_length = lengths[segment + 1] - lengths[segment]
        fraction = 0.0 if segment_length <= 1e-9 else (target - lengths[segment]) / segment_length
        result.append(_interpolate(points[segment], points[segment + 1], fraction))
    return result


def _load_lanelet_centerlines(osm_file: Path) -> dict[str, list[Point]]:
    root = ET.parse(osm_file).getroot()
    nodes: dict[str, Point] = {}
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        if "local_x" not in tags or "local_y" not in tags:
            continue
        nodes[node.get("id", "")] = (
            float(tags["local_x"]),
            float(tags["local_y"]),
            float(tags.get("ele", "0")),
        )

    ways: dict[str, list[Point]] = {}
    for way in root.findall("way"):
        refs = [node.get("ref", "") for node in way.findall("nd")]
        if refs and all(ref in nodes for ref in refs):
            ways[way.get("id", "")] = [nodes[ref] for ref in refs]

    requested = set(SOURCE_LANELET_IDS.values())
    centerlines: dict[str, list[Point]] = {}
    for relation in root.findall("relation"):
        relation_id = relation.get("id", "")
        if relation_id not in requested:
            continue
        members = {
            member.get("role", ""): member.get("ref", "")
            for member in relation.findall("member")
            if member.get("type") == "way"
        }
        try:
            left = ways[members["left"]]
            right = ways[members["right"]]
        except KeyError as error:
            raise ValueError(f"lanelet {relation_id} has incomplete boundaries") from error

        same_direction = _point_distance(left[0], right[0]) + _point_distance(
            left[-1], right[-1]
        )
        opposite_direction = _point_distance(left[0], right[-1]) + _point_distance(
            left[-1], right[0]
        )
        if opposite_direction < same_direction:
            right = list(reversed(right))

        if len(left) != len(right):
            count = max(len(left), len(right))
            left = _resample(left, count)
            right = _resample(right, count)
        centerlines[relation_id] = [
            (
                (left_point[0] + right_point[0]) / 2,
                (left_point[1] + right_point[1]) / 2,
                (left_point[2] + right_point[2]) / 2,
            )
            for left_point, right_point in zip(left, right)
        ]

    missing = requested - centerlines.keys()
    if missing:
        raise ValueError(f"source lanelets were not found: {sorted(missing)}")
    return centerlines


def _connection_key(connection: ET.Element) -> ConnectionKey:
    return (
        connection.get("from", ""),
        connection.get("fromLane", ""),
        connection.get("to", ""),
        connection.get("toLane", ""),
    )


def _edge_lane(root: ET.Element, edge_id: str, lane_index: str) -> tuple[ET.Element, ET.Element]:
    edge = next((item for item in root.findall("edge") if item.get("id") == edge_id), None)
    if edge is None:
        raise ValueError(f"edge {edge_id} was not found in Plain XML")
    lane = next((item for item in edge.findall("lane") if item.get("index") == lane_index), None)
    if lane is None:
        raise ValueError(f"lane {edge_id}_{lane_index} was not found in Plain XML")
    return edge, lane


def _write_xml(tree: ET.ElementTree, path: Path) -> None:
    ET.indent(tree, space="    ")
    tree.write(path, encoding="UTF-8", xml_declaration=True)


def _repair_plain_edges(edge_file: Path, centerlines: dict[str, list[Point]]) -> None:
    tree = ET.parse(edge_file)
    root = tree.getroot()

    _, incoming_lane = _edge_lane(root, "edge_1231", "1")
    incoming_lane.set(
        "shape", _format_shape(centerlines[SOURCE_LANELET_IDS["incoming_left_turn"]])
    )

    outgoing_edge, outgoing_lane = _edge_lane(root, "edge_1235", "0")
    outgoing_lane.set("shape", outgoing_edge.get("shape", ""))

    repeated_edge, repeated_lane = _edge_lane(root, "edge_531", "0")
    source_repeated_shape = centerlines[SOURCE_LANELET_IDS["repeated_point_lane"]]
    repeated_lane.set("shape", _format_shape(source_repeated_shape))
    # The edge centerline comes from the same source, but write it explicitly so
    # this repair remains stable even if the input net is regenerated differently.
    repeated_edge.set("shape", _format_shape(source_repeated_shape))

    for lane_id, source_name in NODE_205_INCOMING_LANELETS.items():
        edge_id, lane_index = lane_id.rsplit("_", 1)
        _, lane = _edge_lane(root, edge_id, lane_index)
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        repaired = _replace_shape_suffix(_parse_shape(lane.get("shape")), source)
        lane.set("shape", _format_shape(repaired))

    for lane_id, source_name in NODE_205_OUTGOING_LANELETS.items():
        edge_id, lane_index = lane_id.rsplit("_", 1)
        _, lane = _edge_lane(root, edge_id, lane_index)
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        repaired = _replace_shape_prefix(
            _parse_shape(lane.get("shape")),
            source,
        )
        lane.set("shape", _format_shape(repaired))

    _write_xml(tree, edge_file)


def _shape_vectors(points: list[Point]) -> list[tuple[float, float]]:
    vectors = []
    for first, second in zip(points, points[1:]):
        vector = (second[0] - first[0], second[1] - first[1])
        if math.hypot(*vector) > 1e-9:
            vectors.append(vector)
    return vectors


def _heading_change(first: tuple[float, float], second: tuple[float, float]) -> float:
    delta = math.atan2(second[1], second[0]) - math.atan2(first[1], first[0])
    return abs((math.degrees(delta) + 180.0) % 360.0 - 180.0)


def _connection_has_discontinuity(
    connection: ET.Element,
    lane_shapes: dict[tuple[str, str], list[Point]],
    max_angle_degrees: float = 45.0,
) -> bool:
    points = _parse_shape(connection.get("shape"))
    connection_vectors = _shape_vectors(points)
    if any(
        _heading_change(first, second) > max_angle_degrees
        for first, second in zip(connection_vectors, connection_vectors[1:])
    ):
        return True

    incoming_vectors = _shape_vectors(
        lane_shapes.get((connection.get("from", ""), connection.get("fromLane", "")), [])
    )
    outgoing_vectors = _shape_vectors(
        lane_shapes.get((connection.get("to", ""), connection.get("toLane", "")), [])
    )
    if (
        incoming_vectors
        and connection_vectors
        and _heading_change(incoming_vectors[-1], connection_vectors[0])
        > max_angle_degrees
    ):
        return True
    return bool(
        outgoing_vectors
        and connection_vectors
        and _heading_change(connection_vectors[-1], outgoing_vectors[0])
        > max_angle_degrees
    )


def _repair_plain_connections(
    edge_file: Path,
    connection_file: Path,
    centerlines: dict[str, list[Point]],
) -> None:
    edge_root = ET.parse(edge_file).getroot()
    lane_shapes = {
        (edge.get("id", ""), lane.get("index", "")): _parse_shape(lane.get("shape"))
        for edge in edge_root.findall("edge")
        for lane in edge.findall("lane")
    }

    tree = ET.parse(connection_file)
    root = tree.getroot()
    connections = {
        _connection_key(connection): connection for connection in root.findall("connection")
    }

    required = (
        PRESERVED_CONNECTIONS
        | {TARGET_CONNECTION, NODE_825_VALID_CONNECTION}
        | set(NODE_205_CONNECTION_LANELETS)
        | NODE_825_INVALID_CONNECTIONS
    )
    missing = required - connections.keys()
    if missing:
        raise ValueError(f"required connections were not found: {sorted(missing)}")

    for key in NODE_825_INVALID_CONNECTIONS:
        root.remove(connections.pop(key))

    preserved_shapes = {
        key: connections[key].get("shape", "") for key in PRESERVED_CONNECTIONS
    }
    regenerated_keys = {
        key
        for key, connection in connections.items()
        if _connection_has_discontinuity(connection, lane_shapes)
    }
    for key in regenerated_keys:
        connections[key].attrib.pop("shape", None)
        connections[key].attrib.pop("length", None)

    for key, shape in preserved_shapes.items():
        points = _parse_shape(shape)
        connections[key].set("shape", shape)
        connections[key].set("length", f"{_polyline_length(points):.3f}")

    turn_points = centerlines[SOURCE_LANELET_IDS["left_turn_connector"]]
    connections[TARGET_CONNECTION].set("shape", _format_shape(turn_points))
    connections[TARGET_CONNECTION].set("length", f"{_polyline_length(turn_points):.3f}")

    for key, (source_name, _) in NODE_205_CONNECTION_LANELETS.items():
        points = centerlines[SOURCE_LANELET_IDS[source_name]]
        connection = connections[key]
        connection.set("shape", _format_shape(points))
        connection.set("length", f"{_polyline_length(points):.3f}")

    _write_xml(tree, connection_file)
    print(f"regenerating {len(regenerated_keys)} discontinuous connection shapes")
    print(
        f"removed {len(NODE_825_INVALID_CONNECTIONS)} invalid node_825 connections"
    )


def _point_at_distance(points: list[Point], distance: float) -> tuple[Point, int]:
    """Return a point and the following source index at a 2D arclength."""

    remaining = distance
    for index, (first, second) in enumerate(zip(points, points[1:])):
        segment_length = _point_distance(first, second)
        if segment_length >= remaining and segment_length > 1e-9:
            return _interpolate(first, second, remaining / segment_length), index + 1
        remaining -= segment_length
    raise ValueError(f"shape is shorter than the requested {distance:.3f} m")


def _trim_shape_start(points: list[Point], distance: float) -> list[Point]:
    new_start, following_index = _point_at_distance(points, distance)
    return [new_start, *points[following_index:]]


def _net_lane(root: ET.Element, lane_id: str) -> ET.Element:
    lane = next((item for item in root.iter("lane") if item.get("id") == lane_id), None)
    if lane is None:
        raise ValueError(f"lane {lane_id} was not found in rebuilt net")
    return lane


def _net_connection(
    root: ET.Element, from_edge: str, from_lane: str, to_edge: str, to_lane: str
) -> ET.Element:
    connection = next(
        (
            item
            for item in root.findall("connection")
            if _connection_key(item) == (from_edge, from_lane, to_edge, to_lane)
        ),
        None,
    )
    if connection is None:
        raise ValueError(
            f"connection {from_edge}_{from_lane}->{to_edge}_{to_lane} was not found"
        )
    return connection


def _set_shape(element: ET.Element, points: list[Point]) -> None:
    element.set("shape", _format_shape(points))
    element.set("length", f"{_polyline_length(points):.3f}")


def _restore_node_205_source_chains(
    root: ET.Element,
    source_root: ET.Element,
    centerlines: dict[str, list[Point]],
) -> None:
    """Undo netconvert's asymmetric clipping at the edge_1384 junction."""

    for lane_id, source_name in NODE_205_INCOMING_LANELETS.items():
        source_lane = _net_lane(source_root, lane_id)
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        repaired = _replace_shape_suffix(
            _parse_shape(source_lane.get("shape")),
            source,
        )
        _set_shape(_net_lane(root, lane_id), repaired)

    for lane_id, source_name in NODE_205_OUTGOING_LANELETS.items():
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        source_lane = _net_lane(source_root, lane_id)
        repaired = _replace_shape_prefix(
            _parse_shape(source_lane.get("shape")),
            source,
        )
        _set_shape(_net_lane(root, lane_id), repaired)

    for key, (source_name, expected_via) in NODE_205_CONNECTION_LANELETS.items():
        from_edge, from_lane, to_edge, to_lane = key
        connection = _net_connection(root, from_edge, from_lane, to_edge, to_lane)
        if connection.get("via") != expected_via:
            raise RuntimeError(
                f"{from_edge}_{from_lane}->{to_edge}_{to_lane} via changed from "
                f"{expected_via} to {connection.get('via')}"
            )

        connector = centerlines[SOURCE_LANELET_IDS[source_name]]
        _set_shape(connection, connector)
        _set_shape(_net_lane(root, expected_via), connector)

        internal_edge, internal_lane = expected_via.rsplit("_", 1)
        downstream = _net_connection(
            root, internal_edge, internal_lane, to_edge, to_lane
        )
        outgoing = _parse_shape(_net_lane(root, f"{to_edge}_{to_lane}").get("shape"))
        second_point, _ = _point_at_distance(outgoing, 0.25)
        _set_shape(downstream, [outgoing[0], second_point])


def _cubic_bezier(
    start: Point,
    start_tangent: tuple[float, float],
    end: Point,
    end_tangent: tuple[float, float],
    steps: int = 24,
) -> list[Point]:
    chord = _point_distance(start, end)
    if chord <= 1e-6:
        return [start, end]
    handle = min(chord * 0.2, 2.0)
    start_length = math.hypot(*start_tangent)
    end_length = math.hypot(*end_tangent)
    first_control = (
        start[0] + start_tangent[0] / start_length * handle,
        start[1] + start_tangent[1] / start_length * handle,
        start[2] + (end[2] - start[2]) / 3,
    )
    second_control = (
        end[0] - end_tangent[0] / end_length * handle,
        end[1] - end_tangent[1] / end_length * handle,
        start[2] + (end[2] - start[2]) * 2 / 3,
    )

    result = []
    for index in range(steps + 1):
        fraction = index / steps
        inverse = 1 - fraction
        weights = (
            inverse**3,
            3 * inverse**2 * fraction,
            3 * inverse * fraction**2,
            fraction**3,
        )
        result.append(
            tuple(
                weights[0] * start[axis]
                + weights[1] * first_control[axis]
                + weights[2] * second_control[axis]
                + weights[3] * end[axis]
                for axis in range(3)
            )
        )
    return result  # type: ignore[return-value]


def _smooth_internal_lane_hooks(
    root: ET.Element, source_root: ET.Element
) -> tuple[int, int]:
    lane_elements = {
        (edge.get("id", ""), lane.get("index", "")): lane
        for edge in root.findall("edge")
        for lane in edge.findall("lane")
    }
    smoothed = 0
    restored = 0
    for connection in root.findall("connection"):
        from_edge = connection.get("from", "")
        to_edge = connection.get("to", "")
        via = connection.get("via")
        if not via or from_edge.startswith(":") or to_edge.startswith(":"):
            continue

        internal_lane = _net_lane(root, via)
        internal_vectors = _shape_vectors(_parse_shape(internal_lane.get("shape")))
        if not any(
            _heading_change(first, second) > 45.0
            for first, second in zip(internal_vectors, internal_vectors[1:])
        ):
            continue

        incoming_lane = lane_elements[(from_edge, connection.get("fromLane", ""))]
        outgoing_lane = lane_elements[(to_edge, connection.get("toLane", ""))]
        incoming = _parse_shape(incoming_lane.get("shape"))
        outgoing = _parse_shape(outgoing_lane.get("shape"))
        incoming_vectors = _shape_vectors(incoming)
        outgoing_vectors = _shape_vectors(outgoing)
        if not incoming_vectors or not outgoing_vectors:
            continue

        replacement = _cubic_bezier(
            incoming[-1],
            incoming_vectors[-1],
            outgoing[0],
            outgoing_vectors[0],
        )
        rounded_vectors = _shape_vectors(_parse_shape(_format_shape(replacement)))
        if any(
            _heading_change(first, second) > 45.0
            for first, second in zip(rounded_vectors, rounded_vectors[1:])
        ):
            source_lane = _net_lane(source_root, via)
            internal_lane.set("shape", source_lane.get("shape", ""))
            internal_lane.set("length", source_lane.get("length", ""))
            connection.attrib.pop("shape", None)
            connection.attrib.pop("length", None)
            restored += 1
            continue

        _set_shape(internal_lane, replacement)
        _set_shape(connection, replacement)
        smoothed += 1
    return smoothed, restored


def _restore_reversed_short_internal_lanes(
    root: ET.Element, source_root: ET.Element
) -> int:
    """Restore malformed direct via stubs from the Lanelet2-derived source net."""

    lanes = {
        lane.get("id", ""): lane
        for edge in root.findall("edge")
        for lane in edge.findall("lane")
    }
    source_lanes = {
        lane.get("id", ""): lane
        for edge in source_root.findall("edge")
        for lane in edge.findall("lane")
    }
    connections = list(root.findall("connection"))
    source_connections: dict[ConnectionKey, list[ET.Element]] = {}
    for connection in source_root.findall("connection"):
        source_connections.setdefault(_connection_key(connection), []).append(connection)

    restored = 0
    for owner in connections:
        from_edge = owner.get("from", "")
        to_edge = owner.get("to", "")
        via = owner.get("via")
        if not via or from_edge.startswith(":") or to_edge.startswith(":"):
            continue
        try:
            internal_edge, internal_lane_index = via.rsplit("_", 1)
        except ValueError:
            continue

        downstream = [
            connection
            for connection in connections
            if connection.get("from") == internal_edge
            and connection.get("fromLane") == internal_lane_index
        ]
        if len(downstream) != 1 or downstream[0].get("via"):
            # Split-junction movements have another internal via and do not own
            # the complete source connection shape.
            continue
        if (
            downstream[0].get("to") != to_edge
            or downstream[0].get("toLane") != owner.get("toLane")
        ):
            continue

        internal_lane = lanes.get(via)
        if internal_lane is None:
            continue
        owner_shape = _parse_shape(owner.get("shape"))
        rebuilt_shape = _parse_shape(internal_lane.get("shape"))
        if len(owner_shape) != 2 or len(rebuilt_shape) != 2:
            continue
        try:
            owner_length = float(owner.get("length", "nan"))
            rebuilt_length = float(internal_lane.get("length", "nan"))
        except ValueError:
            continue
        if not (
            math.isclose(owner_length, 0.250, abs_tol=1e-9)
            and math.isclose(rebuilt_length, 0.250, abs_tol=1e-9)
        ):
            continue

        owner_vectors = _shape_vectors(owner_shape)
        rebuilt_vectors = _shape_vectors(rebuilt_shape)
        malformed = _polyline_length(rebuilt_shape) <= 1e-9 or (
            owner_vectors
            and rebuilt_vectors
            and _heading_change(owner_vectors[0], rebuilt_vectors[0]) > 90.0
        )
        if not malformed:
            continue

        source_owner_candidates = source_connections.get(_connection_key(owner), [])
        source_lane = source_lanes.get(via)
        if len(source_owner_candidates) != 1 or source_lane is None:
            raise RuntimeError(f"{via} has no unique Lanelet2-derived source geometry")
        source_owner = source_owner_candidates[0]
        source_owner_shape = _parse_shape(source_owner.get("shape"))
        source_lane_shape = _parse_shape(source_lane.get("shape"))
        try:
            source_owner_length = float(source_owner.get("length", "nan"))
            source_lane_length = float(source_lane.get("length", "nan"))
        except ValueError as error:
            raise RuntimeError(f"{via} has invalid source length") from error

        safe_source = (
            source_owner.get("via") == via
            and len(source_owner_shape) == 2
            and len(source_lane_shape) == 2
            and source_owner_shape == owner_shape
            and source_lane_shape == owner_shape
            and math.isclose(source_owner_length, 0.250, abs_tol=1e-9)
            and math.isclose(source_lane_length, 0.250, abs_tol=1e-9)
            and _point_distance(rebuilt_shape[0], source_lane_shape[0]) <= 0.002
            and bool(_shape_vectors(source_lane_shape))
        )
        if not safe_source:
            raise RuntimeError(
                f"{via} is malformed but its Lanelet2-derived source is ambiguous"
            )

        internal_lane.set("shape", source_lane.get("shape", ""))
        internal_lane.set("length", source_lane.get("length", ""))
        restored += 1
    return restored


def _postprocess_rebuilt_net(
    net_file: Path, source_net: Path, centerlines: dict[str, list[Point]]
) -> None:
    """Restore exact source boundaries after netconvert's junction clipping.

    netconvert extends custom lanes to the old generated junction polygons. A
    final XML pass is therefore required for the two source-backed repairs.
    Outgoing lanes are shortened by 0.25 m so the internal lane and outgoing
    lane meet at the same point without a zero-length or reversed segment.
    """

    tree = ET.parse(net_file)
    root = tree.getroot()
    source_root = ET.parse(source_net).getroot()

    incoming = centerlines[SOURCE_LANELET_IDS["incoming_left_turn"]]
    turn = centerlines[SOURCE_LANELET_IDS["left_turn_connector"]]
    _set_shape(_net_lane(root, "edge_1231_1"), incoming)
    _set_shape(_net_lane(root, ":node_119_1_0"), turn)
    _set_shape(
        _net_connection(root, "edge_1231", "1", "edge_1235", "0"), turn
    )

    outgoing_lane = _net_lane(root, "edge_1235_0")
    outgoing_edge = next(
        edge for edge in root.findall("edge") if edge.get("id") == "edge_1235"
    )
    outgoing = _parse_shape(outgoing_edge.get("shape"))
    trimmed_outgoing = _trim_shape_start(outgoing, 0.25)
    _set_shape(outgoing_lane, trimmed_outgoing)
    target_second_stage = _net_connection(
        root, ":node_119_1", "0", "edge_1235", "0"
    )
    _set_shape(target_second_stage, [turn[-1], trimmed_outgoing[0]])

    repeated_lane = centerlines[SOURCE_LANELET_IDS["repeated_point_lane"]]
    _set_shape(_net_lane(root, "edge_531_0"), repeated_lane)
    edge_95_lane = _net_lane(root, "edge_95_0")
    edge_95 = _parse_shape(edge_95_lane.get("shape"))
    trimmed_edge_95 = _trim_shape_start(edge_95, 0.25)
    _set_shape(edge_95_lane, trimmed_edge_95)

    short_connector = [repeated_lane[-1], trimmed_edge_95[0]]
    _set_shape(_net_lane(root, ":ia_2017578_3_0"), short_connector)
    _set_shape(
        _net_connection(root, "edge_531", "0", "edge_95", "0"), short_connector
    )
    edge_95_second_point, _ = _point_at_distance(trimmed_edge_95, 0.25)
    _set_shape(
        _net_connection(root, ":ia_2017578_3", "0", "edge_95", "0"),
        [trimmed_edge_95[0], edge_95_second_point],
    )

    smoothed, restored = _smooth_internal_lane_hooks(root, source_root)
    short_restored = _restore_reversed_short_internal_lanes(root, source_root)
    _restore_node_205_source_chains(root, source_root, centerlines)
    print(f"smoothed {smoothed} internal lane hooks; restored {restored} ambiguous hooks")
    print(
        f"restored {short_restored} reversed or degenerate short direct internal lanes"
    )

    _write_xml(tree, net_file)


def _run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def _external_edge_ids(net_file: Path) -> set[str]:
    root = ET.parse(net_file).getroot()
    return {
        edge.get("id", "")
        for edge in root.findall("edge")
        if edge.get("function") != "internal"
    }


def _normal_connection_keys(net_file: Path) -> set[ConnectionKey]:
    root = ET.parse(net_file).getroot()
    return {
        _connection_key(connection)
        for connection in root.findall("connection")
        if not connection.get("from", "").startswith(":")
        and not connection.get("to", "").startswith(":")
    }


def _tls_link_assignments(net_file: Path) -> set[tuple[str | None, ...]]:
    root = ET.parse(net_file).getroot()
    return {
        (
            connection.get("from"),
            connection.get("fromLane"),
            connection.get("to"),
            connection.get("toLane"),
            connection.get("tl"),
            connection.get("linkIndex"),
        )
        for connection in root.findall("connection")
        if connection.get("tl") is not None
    }


def _tls_signatures(net_file: Path) -> set[tuple[object, ...]]:
    root = ET.parse(net_file).getroot()
    signatures = set()
    for tls in root.findall("tlLogic"):
        phases = tuple(tuple(sorted(phase.attrib.items())) for phase in tls.findall("phase"))
        signatures.add(
            (
                tls.get("id", ""),
                tls.get("type", ""),
                tls.get("programID", ""),
                tls.get("offset", ""),
                phases,
            )
        )
    return signatures


def _assert_topology_preserved(source_net: Path, output_net: Path) -> None:
    if _external_edge_ids(source_net) != _external_edge_ids(output_net):
        raise RuntimeError("external edge IDs changed while rebuilding the net")
    source_connections = _normal_connection_keys(source_net)
    output_connections = _normal_connection_keys(output_net)
    expected_connections = source_connections - NODE_825_INVALID_CONNECTIONS
    if output_connections != expected_connections:
        raise RuntimeError(
            "normal edge connection topology changed beyond the node_825 repair"
        )
    if _tls_link_assignments(source_net) != _tls_link_assignments(output_net):
        raise RuntimeError("traffic-light link assignments changed while rebuilding the net")
    if _tls_signatures(source_net) != _tls_signatures(output_net):
        raise RuntimeError("traffic-light programs changed while rebuilding the net")


def _assert_repaired_geometry(output_net: Path, centerlines: dict[str, list[Point]]) -> None:
    root = ET.parse(output_net).getroot()
    expected = {
        "edge_1231_1": centerlines[SOURCE_LANELET_IDS["incoming_left_turn"]],
        ":node_119_1_0": centerlines[SOURCE_LANELET_IDS["left_turn_connector"]],
        "edge_531_0": centerlines[SOURCE_LANELET_IDS["repeated_point_lane"]],
    }
    for lane_id, expected_points in expected.items():
        actual_points = _parse_shape(_net_lane(root, lane_id).get("shape"))
        expected_start = tuple(round(value, 3) for value in expected_points[0])
        expected_end = tuple(round(value, 3) for value in expected_points[-1])
        if actual_points[0] != expected_start:
            raise RuntimeError(f"{lane_id} does not start at the Lanelet2 source point")
        if actual_points[-1] != expected_end:
            raise RuntimeError(f"{lane_id} does not end at the Lanelet2 source point")

    for lane_id, source_name in NODE_205_INCOMING_LANELETS.items():
        actual = _parse_shape(_net_lane(root, lane_id).get("shape"))
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        expected_end = tuple(round(value, 3) for value in source[-1])
        if actual[-1] != expected_end:
            raise RuntimeError(f"{lane_id} does not end at its Lanelet2 source point")

    for lane_id, source_name in NODE_205_OUTGOING_LANELETS.items():
        actual = _parse_shape(_net_lane(root, lane_id).get("shape"))
        source = centerlines[SOURCE_LANELET_IDS[source_name]]
        expected_start = tuple(round(value, 3) for value in source[0])
        if actual[0] != expected_start:
            raise RuntimeError(f"{lane_id} does not start at its Lanelet2 source point")

    for key, (source_name, expected_via) in NODE_205_CONNECTION_LANELETS.items():
        from_edge, from_lane, to_edge, to_lane = key
        connection = _net_connection(root, from_edge, from_lane, to_edge, to_lane)
        expected_shape = [
            tuple(round(value, 3) for value in point)
            for point in centerlines[SOURCE_LANELET_IDS[source_name]]
        ]
        if connection.get("via") != expected_via:
            raise RuntimeError(
                f"{from_edge}_{from_lane}->{to_edge}_{to_lane} via is not "
                f"{expected_via}"
            )
        if _parse_shape(connection.get("shape")) != expected_shape:
            raise RuntimeError(
                f"{from_edge}_{from_lane}->{to_edge}_{to_lane} is not source-backed"
            )
        if _parse_shape(_net_lane(root, expected_via).get("shape")) != expected_shape:
            raise RuntimeError(f"{expected_via} is not source-backed")

    node_825_incoming_end = centerlines[SOURCE_LANELET_IDS["node_825_incoming"]][-1]
    node_825_valid_start = centerlines[
        SOURCE_LANELET_IDS["node_825_valid_outgoing"]
    ][0]
    node_825_invalid_starts = (
        centerlines[SOURCE_LANELET_IDS["node_825_invalid_outgoing_0"]][0],
        centerlines[SOURCE_LANELET_IDS["node_825_invalid_outgoing_1"]][0],
    )
    if _point_distance(node_825_incoming_end, node_825_valid_start) >= 1e-6:
        raise RuntimeError("node_825 valid Lanelet2 successor is not continuous")
    if any(
        _point_distance(node_825_incoming_end, point) <= 8.0
        for point in node_825_invalid_starts
    ):
        raise RuntimeError("node_825 rejected Lanelet2 successor is unexpectedly nearby")

    output_connections = _normal_connection_keys(output_net)
    if NODE_825_INVALID_CONNECTIONS & output_connections:
        raise RuntimeError("invalid node_825 connections remain in the rebuilt net")
    if NODE_825_VALID_CONNECTION not in output_connections:
        raise RuntimeError("valid node_825 Lanelet2 successor was removed")


def main() -> None:
    args = _parse_args()
    source_net = args.source_net.resolve()
    source_osm = args.source_osm.resolve()
    output_net = args.output_net.resolve()
    if source_net == output_net:
        raise ValueError("--output-net must differ from --source-net")
    if not source_net.is_file() or not source_osm.is_file():
        raise FileNotFoundError("source net and source OSM must both exist")

    centerlines = _load_lanelet_centerlines(source_osm)
    output_net.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="odaiba_sumo_repair_") as temp_directory:
        plain_prefix = Path(temp_directory) / "network"
        _run(
            [
                args.netconvert,
                "--sumo-net-file",
                str(source_net),
                "--plain-output-prefix",
                str(plain_prefix),
                "--plain-output.lanes",
                "true",
                "--precision",
                "3",
            ]
        )

        edge_file = plain_prefix.with_suffix(".edg.xml")
        connection_file = plain_prefix.with_suffix(".con.xml")
        config_file = plain_prefix.with_suffix(".netccfg")
        _repair_plain_edges(edge_file, centerlines)
        _repair_plain_connections(edge_file, connection_file, centerlines)

        _run(
            [
                args.netconvert,
                "--configuration-file",
                str(config_file),
                "--output-file",
                str(output_net),
                "--precision",
                "3",
            ]
        )

    _postprocess_rebuilt_net(output_net, source_net, centerlines)
    _assert_topology_preserved(source_net, output_net)
    _assert_repaired_geometry(output_net, centerlines)
    print(f"repaired net: {output_net}")
    print("source net was read-only and was not modified")


if __name__ == "__main__":
    main()
