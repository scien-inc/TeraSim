"""Geometry continuity checks for SUMO network lane and connection shapes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class GeometryDiscontinuity:
    kind: str
    object_id: str
    location: str
    angle_degrees: float

    @property
    def key(self) -> tuple[str, str, str]:
        return (self.kind, self.object_id, self.location)


def parse_shape(shape: str | None) -> tuple[tuple[float, ...], ...]:
    if not shape:
        return ()
    return tuple(tuple(float(value) for value in point.split(",")) for point in shape.split())


def _vector(
    start: tuple[float, ...], end: tuple[float, ...]
) -> tuple[float, float] | None:
    vector = (end[0] - start[0], end[1] - start[1])
    if math.hypot(*vector) <= 1e-9:
        return None
    return vector


def _heading_change_degrees(
    first: tuple[float, float], second: tuple[float, float]
) -> float:
    delta = math.atan2(second[1], second[0]) - math.atan2(first[1], first[0])
    return abs((math.degrees(delta) + 180.0) % 360.0 - 180.0)


def _shape_vectors(
    points: tuple[tuple[float, ...], ...],
) -> list[tuple[int, tuple[float, float]]]:
    vectors = []
    for index, (start, end) in enumerate(zip(points, points[1:])):
        vector = _vector(start, end)
        if vector is not None:
            vectors.append((index, vector))
    return vectors


def _append_shape_discontinuities(
    issues: list[GeometryDiscontinuity],
    kind: str,
    object_id: str,
    points: tuple[tuple[float, ...], ...],
    max_angle_degrees: float,
) -> None:
    vectors = _shape_vectors(points)
    for (first_index, first), (second_index, second) in zip(vectors, vectors[1:]):
        angle = _heading_change_degrees(first, second)
        if angle > max_angle_degrees:
            issues.append(
                GeometryDiscontinuity(
                    kind=kind,
                    object_id=object_id,
                    location=f"vertex:{first_index + 1}:{second_index}",
                    angle_degrees=angle,
                )
            )


def _terminal_vector(
    points: tuple[tuple[float, ...], ...], *, first: bool
) -> tuple[float, float] | None:
    vectors = _shape_vectors(points)
    if not vectors:
        return None
    return vectors[0][1] if first else vectors[-1][1]


def find_geometry_discontinuities(
    net_file: str | Path,
    *,
    max_angle_degrees: float = 45.0,
) -> list[GeometryDiscontinuity]:
    """Scan every lane shape and connection boundary in a SUMO net."""

    root = ET.parse(net_file).getroot()
    lane_shapes: dict[tuple[str, str], tuple[tuple[float, ...], ...]] = {}
    issues: list[GeometryDiscontinuity] = []

    for edge in root.findall("edge"):
        edge_id = edge.get("id", "")
        for lane in edge.findall("lane"):
            lane_id = lane.get("id", "")
            lane_index = lane.get("index", "")
            points = parse_shape(lane.get("shape"))
            lane_shapes[(edge_id, lane_index)] = points
            _append_shape_discontinuities(
                issues,
                "lane_shape",
                lane_id,
                points,
                max_angle_degrees,
            )

    for connection in root.findall("connection"):
        from_edge = connection.get("from", "")
        from_lane = connection.get("fromLane", "")
        to_edge = connection.get("to", "")
        to_lane = connection.get("toLane", "")
        connection_id = f"{from_edge}_{from_lane}->{to_edge}_{to_lane}"
        connection_points = parse_shape(connection.get("shape"))

        _append_shape_discontinuities(
            issues,
            "connection_shape",
            connection_id,
            connection_points,
            max_angle_degrees,
        )

        incoming_points = lane_shapes.get((from_edge, from_lane), ())
        outgoing_points = lane_shapes.get((to_edge, to_lane), ())
        incoming_heading = _terminal_vector(incoming_points, first=False)
        outgoing_heading = _terminal_vector(outgoing_points, first=True)
        connection_first = _terminal_vector(connection_points, first=True)
        connection_last = _terminal_vector(connection_points, first=False)

        if incoming_heading is not None and connection_first is not None:
            angle = _heading_change_degrees(incoming_heading, connection_first)
            if angle > max_angle_degrees:
                issues.append(
                    GeometryDiscontinuity(
                        kind="connection_boundary",
                        object_id=connection_id,
                        location="entry",
                        angle_degrees=angle,
                    )
                )

        if connection_last is not None and outgoing_heading is not None:
            angle = _heading_change_degrees(connection_last, outgoing_heading)
            if angle > max_angle_degrees:
                issues.append(
                    GeometryDiscontinuity(
                        kind="connection_boundary",
                        object_id=connection_id,
                        location="exit",
                        angle_degrees=angle,
                    )
                )

    return issues


def format_discontinuities(issues: Iterable[GeometryDiscontinuity]) -> str:
    return "\n".join(
        f"{issue.kind} {issue.object_id} {issue.location}: "
        f"{issue.angle_degrees:.3f} deg"
        for issue in sorted(
            issues,
            key=lambda issue: (
                issue.kind,
                issue.object_id,
                issue.location,
                issue.angle_degrees,
            ),
        )
    )
