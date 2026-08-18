"""Focused edge_426 Phase-A/Phase-B lane-change synchronization checks."""

from __future__ import annotations

import json
import os
import subprocess
import textwrap
import types
from functools import lru_cache
from pathlib import Path
from xml.etree import ElementTree

import pytest


WAIT_CYCLES = 3
FIELD_LEFT_STEPS = 65
OUTSIDE_CYCLES = 60
STEP_LENGTH = 0.05
POSITION_TOLERANCE = 1e-6
ANGLE_OFFSET_TOLERANCE = 1e-6
TARGET_CENTER_TOLERANCE = 1e-4
FIELD_COMPLETION_TOLERANCE = 0.05 + 1e-6
SUMO_LATERAL_SPEED_LIMIT = 1.0 + 1e-6


def _probe_path() -> Path:
    value = os.environ.get("SUMO_EXTERNAL_STATE_RIGHT_REVERSAL_PROBE")
    if not value:
        pytest.skip("SUMO_EXTERNAL_STATE_RIGHT_REVERSAL_PROBE is not configured")
    path = Path(value)
    if not path.is_file():
        pytest.skip(f"right-reversal probe binary is not available: {path}")
    return path


def _network_path() -> Path:
    path = (
        Path(__file__).resolve().parents[2]
        / "examples/maps/odaiba_ll2/tlmappings_0708/network.repaired_geometry.net.xml"
    )
    if not path.is_file():
        pytest.skip("Odaiba repaired-geometry network is not available")
    return path


def _write_route(
    tmp_path: Path,
    *,
    vehicle_id: str,
    depart_lane: int,
) -> Path:
    path = tmp_path / f"edge426-{vehicle_id}-lane{depart_lane}.rou.xml"
    path.write_text(
        textwrap.dedent(
            f"""\
            <routes>
                <vType id="car" accel="2.6" decel="4.5" emergencyDecel="9"
                       sigma="0" length="5" width="1.8" maxSpeed="16.667"
                       laneChangeModel="SL2015"/>
                <route id="route"
                       edges="edge_426 edge_432 edge_427 edge_52 edge_54 edge_59 edge_255"/>
                <vehicle id="{vehicle_id}" type="car" route="route" depart="0"
                         departLane="{depart_lane}" departPos="45" departSpeed="9"/>
            </routes>
            """
        ),
        encoding="utf-8",
    )
    return path


def _angle_difference(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)


def _raw_intent(snapshot: dict) -> str:
    wants_left = bool(int(snapshot["left_state"]) & 2)
    wants_right = bool(int(snapshot["right_state"]) & 4)
    if wants_left == wants_right:
        return "none"
    return "left" if wants_left else "right"


def _annotate_service_actions(records: list[dict], monkeypatch) -> None:
    """Feed every recorded raw LCA decision through the production exporter."""
    from terasim_service.plugins import cosim as plugin_module

    current: dict[str, dict] = {}

    def get_lane_change_state(_vehicle_id: str, direction: int) -> tuple[int, int]:
        snapshot = current["snapshot"]
        key = "left_state" if direction == 1 else "right_state"
        return 0, int(snapshot[key])

    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(
            constants=types.SimpleNamespace(LCA_LEFT=2, LCA_RIGHT=4),
            vehicle=types.SimpleNamespace(
                getLaneChangeState=get_lane_change_state,
            ),
        ),
    )
    plugins: dict[str, object] = {}
    for snapshot in records:
        scenario = snapshot["scenario"]
        if scenario not in plugins:
            plugin = plugin_module.TeraSimCoSimPlugin.__new__(
                plugin_module.TeraSimCoSimPlugin
            )
            plugin.external_state_lane_change_maneuvers = {}
            plugins[scenario] = plugin
        current["snapshot"] = snapshot
        intent, target_lane_id = plugins[scenario]._get_sumo_lane_change_action(
            snapshot["vehicle_id"], snapshot["primary_lane"]
        )
        snapshot["raw_intent"] = _raw_intent(snapshot)
        snapshot["service_intent"] = intent
        snapshot["service_target_lane"] = target_lane_id


@lru_cache(maxsize=1)
def _lane_shapes() -> dict[str, list[tuple[float, float]]]:
    root = ElementTree.parse(_network_path()).getroot()
    return {
        lane.get("id", ""): [
            tuple(float(value) for value in point.split(",")[:2])
            for point in lane.get("shape", "").split()
        ]
        for lane in root.iter("lane")
        if lane.get("id") and lane.get("shape")
    }


def _annotate_world_lateral_action(phase_a: dict, phase_b: dict) -> dict:
    from terasim_service.utils.sumo_lane_geometry import (
        build_external_state_lateral_action_lookahead,
        compile_lane_shapes,
    )

    result = build_external_state_lateral_action_lookahead(
        compile_lane_shapes([_lane_shapes()[phase_b["primary_lane"]]]),
        (phase_b["position_x"], phase_b["position_y"]),
        7.0,
        lateral_speed=phase_b["speed_lat"],
        desired_speed=7.0,
        phase_a_position=(phase_a["position_x"], phase_a["position_y"]),
        phase_step_length=STEP_LENGTH,
    )
    for key in (
        "route_tangent_x",
        "route_tangent_y",
        "world_left_normal_x",
        "world_left_normal_y",
        "phase_b_lateral_delta",
        "expected_phase_b_lateral_distance",
        "world_lateral_speed",
        "lateral_displacement",
    ):
        phase_b[f"service_{key}"] = result.get(key)
    phase_b["service_lookahead_valid"] = result["valid"]
    phase_b["service_lookahead_error"] = result["error"]
    return result


def _write_artifact(vehicle_id: str, strict_lane_hint: bool, records: list[dict]) -> None:
    value = os.environ.get("SUMO_EXTERNAL_STATE_RIGHT_REVERSAL_ARTIFACT_DIR")
    if not value:
        return
    directory = Path(value)
    directory.mkdir(parents=True, exist_ok=True)
    mode = "strict" if strict_lane_hint else "standard"
    output = "".join(json.dumps(record, sort_keys=True) + "\n" for record in records)
    (directory / f"{vehicle_id}-{mode}.ndjson").write_text(output, encoding="utf-8")


def _one(records: list[dict], scenario: str, stage: str, cycle: int = 0) -> dict:
    matches = [
        record
        for record in records
        if record["scenario"] == scenario
        and record["stage"] == stage
        and record["cycle"] == cycle
    ]
    assert len(matches) == 1
    return matches[0]


def _stage(records: list[dict], scenario: str, stage: str) -> list[dict]:
    return sorted(
        (
            record
            for record in records
            if record["scenario"] == scenario and record["stage"] == stage
        ),
        key=lambda record: record["cycle"],
    )


@pytest.mark.parametrize("strict_lane_hint", [False, True], ids=["standard", "strict"])
@pytest.mark.parametrize("vehicle_id", ["AV", "BV"])
@pytest.mark.integration
@pytest.mark.requires_sumo
def test_edge426_realized_right_reversal_preserves_sumo_phase_b_decision(
    tmp_path: Path,
    vehicle_id: str,
    strict_lane_hint: bool,
    monkeypatch,
) -> None:
    """CARLA pose is Phase A truth; SUMO remains the Phase B decision source."""
    left_route = _write_route(tmp_path, vehicle_id=vehicle_id, depart_lane=0)
    right_route = _write_route(tmp_path, vehicle_id=vehicle_id, depart_lane=1)
    completed = subprocess.run(
        [
            str(_probe_path()),
            str(_network_path()),
            str(left_route),
            str(right_route),
            vehicle_id,
            str(strict_lane_hint).lower(),
            str(WAIT_CYCLES),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=120,
    )
    records: list[dict] = []
    result = None
    for line in completed.stdout.splitlines():
        if line.startswith("RESULT_JSON="):
            result = json.loads(line.removeprefix("RESULT_JSON="))
        elif line.strip():
            records.append(json.loads(line))
    assert result is not None
    assert records
    _annotate_service_actions(records, monkeypatch)

    assert result["vehicle_id"] == vehicle_id
    assert result["strict_lane_hint"] is strict_lane_hint
    assert result["wait_cycles"] == WAIT_CYCLES
    assert result["field_left_steps"] == FIELD_LEFT_STEPS
    assert result["outside_cycles"] == OUTSIDE_CYCLES
    assert result["left_route_unchanged"] is True
    assert result["right_route_unchanged"] is True
    assert all(record["route_unchanged"] for record in records)
    assert all(record["service_intent"] == record["raw_intent"] for record in records)
    assert all(record["service_target_lane"] == "" for record in records)

    field_authorize = _one(records, "field_trigger", "left_authorize")
    assert field_authorize["primary_lane"] == "edge_426_0"
    assert field_authorize["lcm_target_lane"] == "edge_426_1"
    assert field_authorize["maneuver_distance"] > 0.0

    left_phase_a = _stage(records, "field_trigger", "left_gradual_phase_a")
    left_phase_b = _stage(records, "field_trigger", "left_gradual_phase_b")
    assert len(left_phase_a) == len(left_phase_b) == FIELD_LEFT_STEPS
    assert [record["cycle"] for record in left_phase_a] == list(range(FIELD_LEFT_STEPS))
    assert [record["cycle"] for record in left_phase_b] == list(range(FIELD_LEFT_STEPS))

    previous_primary_lane = field_authorize["primary_lane"]
    primary_lane_transitions: list[int] = []
    for phase_a, phase_b in zip(left_phase_a, left_phase_b):
        # Phase A applies the external pose but does not own lane membership.
        assert phase_a["primary_lane"] == previous_primary_lane
        if phase_b["primary_lane"] != phase_a["primary_lane"]:
            assert (
                phase_a["primary_lane"],
                phase_b["primary_lane"],
            ) == ("edge_426_0", "edge_426_1")
            primary_lane_transitions.append(phase_a["cycle"])
        previous_primary_lane = phase_b["primary_lane"]
        assert phase_a["maneuver_distance"] >= -POSITION_TOLERANCE
        assert abs(phase_b["speed_lat"]) <= SUMO_LATERAL_SPEED_LIMIT
        assert _angle_difference(phase_a["angle"], phase_b["angle"]) < 45.0
    assert len(primary_lane_transitions) == 1

    completed_phase_a_index = next(
        index
        for index, record in enumerate(left_phase_a)
        if abs(record["maneuver_distance"]) <= POSITION_TOLERANCE
    )
    assert completed_phase_a_index > 0
    assert (
        left_phase_a[completed_phase_a_index - 1]["maneuver_distance"]
        > POSITION_TOLERANCE
    )
    for phase_a, phase_b in zip(
        left_phase_a[completed_phase_a_index:],
        left_phase_b[completed_phase_a_index:],
    ):
        assert phase_a["maneuver_distance"] == pytest.approx(
            0.0, abs=POSITION_TOLERANCE
        )
        assert phase_b["maneuver_distance"] == pytest.approx(
            0.0, abs=POSITION_TOLERANCE
        )
        assert phase_a["lcm_target_lane"] == ""
        assert phase_b["lcm_target_lane"] == ""

    left_target_phase_a = left_phase_a[-1]
    assert left_target_phase_a["maneuver_distance"] >= -POSITION_TOLERANCE
    assert (
        abs(left_target_phase_a["maneuver_distance"])
        <= FIELD_COMPLETION_TOLERANCE
    )
    assert abs(left_target_phase_a["target_pos_lat"]) <= FIELD_COMPLETION_TOLERANCE

    left_final = left_phase_b[-1]
    assert left_final["primary_lane"] == "edge_426_1"
    assert abs(left_final["primary_pos_lat"]) <= FIELD_COMPLETION_TOLERANCE
    assert abs(left_final["target_pos_lat"]) <= FIELD_COMPLETION_TOLERANCE

    wait_phase_a = _stage(records, "field_trigger", "wait_phase_a")
    wait_phase_b = _stage(records, "field_trigger", "wait_phase_b")
    assert len(wait_phase_a) == len(wait_phase_b) == WAIT_CYCLES
    assert all(record["primary_lane"] == "edge_426_1" for record in wait_phase_a)
    assert all(record["primary_lane"] == "edge_426_1" for record in wait_phase_b)
    assert all(
        record["speed_lat"] == pytest.approx(0.0, abs=POSITION_TOLERANCE)
        for record in wait_phase_a
    )
    assert all(
        abs(record["maneuver_distance"]) <= FIELD_COMPLETION_TOLERANCE
        for record in wait_phase_a
    )
    assert all(
        abs(record["primary_pos_lat"]) <= FIELD_COMPLETION_TOLERANCE
        for record in wait_phase_b
    )
    assert all(
        abs(record["maneuver_distance"]) <= FIELD_COMPLETION_TOLERANCE
        for record in wait_phase_b
    )
    assert all(
        abs(record["speed_lat"]) <= SUMO_LATERAL_SPEED_LIMIT
        for record in wait_phase_b
    )
    assert all(record["lcm_target_lane"] == "" for record in wait_phase_a)
    assert all(record["lcm_target_lane"] == "" for record in wait_phase_b)
    assert all(
        _angle_difference(phase_a["angle"], phase_b["angle"]) < 45.0
        for phase_a, phase_b in zip(wait_phase_a, wait_phase_b)
    )

    zero_stale_pre = _stage(
        records, "field_trigger", "zero_stale_pre_phase_a"
    )
    zero_stale_phase_a = _stage(
        records, "field_trigger", "zero_stale_phase_a"
    )
    zero_stale_phase_b = _stage(
        records, "field_trigger", "zero_stale_phase_b"
    )
    assert len(zero_stale_pre) == len(zero_stale_phase_a) == len(zero_stale_phase_b) == 3
    first_stale = zero_stale_pre[0]
    assert first_stale["primary_lane"] == "edge_426_1"
    assert first_stale["maneuver_distance"] == pytest.approx(
        0.0, abs=POSITION_TOLERANCE
    )
    assert first_stale["service_intent"] == "none"
    assert first_stale["lcm_target_lane"] == ""
    assert first_stale["shadow_lane"] == ""
    assert first_stale["speed_lat"] == pytest.approx(1.0)
    for phase_a, phase_b in zip(zero_stale_phase_a, zero_stale_phase_b):
        assert phase_a["primary_lane"] == phase_b["primary_lane"] == "edge_426_1"
        assert phase_a["maneuver_distance"] == pytest.approx(
            0.0, abs=POSITION_TOLERANCE
        )
        assert phase_b["maneuver_distance"] == pytest.approx(
            0.0, abs=POSITION_TOLERANCE
        )
        assert phase_a["service_intent"] == phase_b["service_intent"] == "none"
        assert phase_a["lcm_target_lane"] == phase_b["lcm_target_lane"] == ""
        assert phase_a["shadow_lane"] == phase_b["shadow_lane"] == ""
        assert phase_a["speed_lat"] == pytest.approx(0.0, abs=POSITION_TOLERANCE)
        assert phase_b["speed_lat"] == pytest.approx(0.0, abs=POSITION_TOLERANCE)
    zero_stale_angle_offsets = [
        record["angle_offset_degrees"] for record in zero_stale_phase_a
    ]
    assert max(zero_stale_angle_offsets) - min(zero_stale_angle_offsets) < ANGLE_OFFSET_TOLERANCE

    fresh_authorize = _one(records, "fresh_right", "right_authorize")
    assert fresh_authorize["primary_lane"] == "edge_426_1"
    assert fresh_authorize["maneuver_distance"] < 0.0
    assert fresh_authorize["service_intent"] == "right"
    assert fresh_authorize["service_target_lane"] == ""

    right_a1 = _one(records, "fresh_right", "right_a1")
    right_b1 = _one(records, "fresh_right", "right_b1")
    right_a2 = _one(records, "fresh_right", "right_a2")
    right_b2 = _one(records, "fresh_right", "right_b2")
    for phase_a, phase_b in ((right_a1, right_b1), (right_a2, right_b2)):
        assert phase_a["primary_lane"] == "edge_426_1"
        assert phase_b["primary_lane"] == "edge_426_1"
        assert phase_a["maneuver_distance"] < 0.0
        assert _angle_difference(phase_a["angle"], phase_b["angle"]) < 45.0
    assert abs(right_a2["speed_lat"]) > 1.0
    assert abs(right_b1["speed_lat"]) <= SUMO_LATERAL_SPEED_LIMIT
    assert abs(right_b2["speed_lat"]) <= SUMO_LATERAL_SPEED_LIMIT
    assert abs(right_b2["speed_lat"]) < abs(right_a2["speed_lat"]) / 5.0

    frozen_phase_a = _one(records, "fresh_right", "right_frozen_phase_a")
    frozen_phase_b = _one(records, "fresh_right", "right_frozen_phase_b")
    assert frozen_phase_a["primary_lane"] == "edge_426_1"
    assert frozen_phase_b["primary_lane"] == "edge_426_1"
    assert frozen_phase_a["speed_lat"] == pytest.approx(0.0, abs=1e-9)
    assert abs(frozen_phase_b["speed_lat"]) <= SUMO_LATERAL_SPEED_LIMIT
    assert frozen_phase_a["maneuver_distance"] == pytest.approx(
        right_a2["maneuver_distance"], abs=POSITION_TOLERANCE
    )
    assert _angle_difference(frozen_phase_a["angle"], frozen_phase_b["angle"]) < 45.0

    outside_phase_a = _stage(records, "fresh_right", "right_outside_phase_a")
    outside_phase_b = _stage(records, "fresh_right", "right_outside_phase_b")
    assert len(outside_phase_a) == len(outside_phase_b) == OUTSIDE_CYCLES
    assert all(record["primary_lane"] == "edge_426_1" for record in outside_phase_a)
    assert all(record["primary_lane"] == "edge_426_1" for record in outside_phase_b)
    assert all(record["lcm_target_lane"] == "edge_426_0" for record in outside_phase_a)
    assert all(record["lcm_target_lane"] == "edge_426_0" for record in outside_phase_b)
    assert all(
        record["maneuver_distance"] < 0.0
        for record in outside_phase_a
    )
    assert all(record["maneuver_distance"] < 0.0 for record in outside_phase_b)
    assert all(record["service_intent"] == "right" for record in outside_phase_a)
    assert all(record["service_intent"] == "right" for record in outside_phase_b)
    assert all(record["service_target_lane"] == "" for record in outside_phase_a)
    assert all(record["service_target_lane"] == "" for record in outside_phase_b)
    assert all(
        record["speed_lat"] == pytest.approx(0.0, abs=1e-9)
        for record in outside_phase_a[1:]
    )
    assert all(
        record["maneuver_distance"]
        == pytest.approx(outside_phase_a[0]["maneuver_distance"], abs=POSITION_TOLERANCE)
        for record in outside_phase_a[1:]
    )
    assert all(
        abs(record["speed_lat"]) <= SUMO_LATERAL_SPEED_LIMIT
        for record in outside_phase_b
    )
    assert all(
        phase_b["maneuver_distance"] - phase_a["maneuver_distance"]
        == pytest.approx(
            abs(phase_b["speed_lat"]) * STEP_LENGTH,
            abs=POSITION_TOLERANCE,
        )
        for phase_a, phase_b in zip(outside_phase_a, outside_phase_b)
    )
    assert all(
        _angle_difference(phase_a["angle"], phase_b["angle"]) < 45.0
        for phase_a, phase_b in zip(outside_phase_a, outside_phase_b)
    )

    # Compare raw world XY directly. Do not normalize through SUMO's
    # getLateralGeometrySign(), which hid the Odaiba field sign inversion.
    world_lateral_samples = []
    for phase_a, phase_b in (
        list(zip(left_phase_a, left_phase_b))
        + list(zip(outside_phase_a, outside_phase_b))
    ):
        if (
            phase_a["primary_lane"] != phase_b["primary_lane"]
            or abs(phase_b["speed_lat"]) <= 0.05
        ):
            continue
        result = _annotate_world_lateral_action(phase_a, phase_b)
        if abs(result["phase_b_lateral_delta"] or 0.0) <= POSITION_TOLERANCE:
            continue
        assert result["valid"] is True
        assert abs(result["phase_b_lateral_delta"]) == pytest.approx(
            result["expected_phase_b_lateral_distance"],
            abs=POSITION_TOLERANCE,
        )
        assert result["world_lateral_speed"] * result["phase_b_lateral_delta"] > 0.0
        world_lateral_samples.append(result)
    assert world_lateral_samples
    assert any(sample["world_lateral_speed"] > 0.0 for sample in world_lateral_samples)
    assert any(sample["world_lateral_speed"] < 0.0 for sample in world_lateral_samples)

    target_phase_a = _one(records, "fresh_right", "right_target_phase_a")
    target_phase_b = _one(records, "fresh_right", "right_target_phase_b")
    assert target_phase_a["primary_lane"] == "edge_426_1"
    assert target_phase_a["maneuver_distance"] == pytest.approx(
        0.0, abs=POSITION_TOLERANCE
    )
    assert target_phase_a["target_pos_lat"] == pytest.approx(
        0.0, abs=TARGET_CENTER_TOLERANCE
    )

    assert target_phase_b["primary_lane"] == "edge_426_0"
    assert target_phase_b["primary_pos_lat"] == pytest.approx(
        0.0, abs=TARGET_CENTER_TOLERANCE
    )
    assert target_phase_b["target_pos_lat"] == pytest.approx(
        0.0, abs=TARGET_CENTER_TOLERANCE
    )
    assert target_phase_b["maneuver_distance"] == pytest.approx(
        0.0, abs=POSITION_TOLERANCE
    )
    # A completed maneuver must not re-apply the pre-Phase-A lateral speed
    # during the same Phase B primary-lane handover cycle.
    assert target_phase_b["speed_lat"] == pytest.approx(
        0.0, abs=POSITION_TOLERANCE
    )
    target_action = _annotate_world_lateral_action(target_phase_a, target_phase_b)
    assert target_action["valid"] is True
    assert target_action["error"] == ""
    assert target_action["mode"] == "route"
    assert target_phase_b["lcm_target_lane"] == ""
    assert target_phase_b["service_intent"] != "left"
    assert _angle_difference(target_phase_a["angle"], target_phase_b["angle"]) < 45.0

    _write_artifact(vehicle_id, strict_lane_hint, records)
