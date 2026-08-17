"""Assertions for the test-only SUMO/service lane-change angle probe."""

import json
import os
import subprocess
import types
from pathlib import Path

import pytest

PROBE_CYCLES = 250
ANGLE_TOLERANCE = 1e-6
POSITION_TOLERANCE = 1e-6


def _probe_paths() -> tuple[Path, Path, Path]:
    binary_value = os.environ.get("SUMO_EXTERNAL_STATE_ANGLE_PROBE")
    if not binary_value:
        pytest.skip("SUMO_EXTERNAL_STATE_ANGLE_PROBE is not configured")
    binary_path = Path(binary_value)
    fixture_path = Path(
        os.environ.get(
            "SUMO_EXTERNAL_STATE_ANGLE_FIXTURE",
            "/opt/sumo-source/tests/sumo/sublane_model/lateral_speed/"
            "red_light_maxSpeedLatStanding_1",
        )
    )
    network_path = fixture_path / "net.net.xml"
    routes_path = fixture_path / "input_routes.rou.xml"
    if not binary_path.is_file():
        pytest.skip(f"angle probe binary is not available: {binary_path}")
    if not network_path.is_file() or not routes_path.is_file():
        pytest.skip(f"angle probe fixture is not available: {fixture_path}")
    return binary_path, network_path, routes_path


def _angle_difference(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)


def _annotate_service_actions(records: list[dict], monkeypatch) -> None:
    """Run recorded raw SUMO LC states through the production service exporter."""
    from terasim_service.plugins import cosim as plugin_module

    current: dict[str, object] = {}

    def get_lane_change_state(_vehicle_id: str, direction: int) -> tuple[int, int]:
        snapshot = current["snapshot"]
        assert isinstance(snapshot, dict)
        state_key = "left_state" if direction == 1 else "right_state"
        return 0, int(snapshot[state_key])

    def get_lane_index(_vehicle_id: str) -> int:
        snapshot = current["snapshot"]
        assert isinstance(snapshot, dict)
        return int(snapshot["lane_index"])

    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(
            constants=types.SimpleNamespace(LCA_LEFT=2, LCA_RIGHT=4),
            vehicle=types.SimpleNamespace(
                getLaneChangeState=get_lane_change_state,
                getLaneIndex=get_lane_index,
            ),
        ),
    )
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.external_state_lane_change_maneuvers = {}

    def annotate(snapshot: dict, vehicle_id: str) -> None:
        current["snapshot"] = snapshot
        intent, target_lane_id = plugin._get_sumo_lane_change_action(
            vehicle_id, snapshot["primary_lane"]
        )
        snapshot["service_lane_change_intent"] = intent
        snapshot["service_lane_change_target_lane_id"] = target_lane_id

    candidate = records[0]
    annotate(candidate["pure_state"], candidate["pure_vehicle_id"])
    annotate(candidate["feedback_state"], candidate["feedback_vehicle_id"])
    for record in records[1:]:
        for branch in ("pre_phase_a", "phase_a", "phase_b", "pure"):
            annotate(record[branch], record["vehicle_id"])


def _write_artifact(
    records: list[dict],
    lateral_sign: float,
    strict_lane_hint: bool,
) -> None:
    artifact_dir_value = os.environ.get("SUMO_EXTERNAL_STATE_ANGLE_ARTIFACT_DIR")
    if not artifact_dir_value:
        return
    artifact_dir = Path(artifact_dir_value)
    artifact_dir.mkdir(parents=True, exist_ok=True)
    sign_name = "positive" if lateral_sign > 0 else "negative"
    mode_name = "strict" if strict_lane_hint else "standard"
    artifact_path = artifact_dir / f"{sign_name}-{mode_name}.ndjson"
    artifact_path.write_text(
        "".join(
            json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n" for record in records
        ),
        encoding="utf-8",
    )


@pytest.mark.parametrize("strict_lane_hint", [False, True], ids=["standard", "strict"])
@pytest.mark.parametrize("lateral_sign", [-1.0, 1.0], ids=["negative", "positive"])
@pytest.mark.integration
@pytest.mark.requires_sumo
def test_repeated_external_pose_rebases_realized_phase_a_lane_change_state(
    lateral_sign: float,
    strict_lane_hint: bool,
    monkeypatch,
) -> None:
    """A frozen physical pose must not consume or accumulate SUMO LC state."""
    binary_path, network_path, routes_path = _probe_paths()
    completed = subprocess.run(
        [
            str(binary_path),
            str(network_path),
            str(routes_path),
            str(lateral_sign),
            str(strict_lane_hint).lower(),
            str(PROBE_CYCLES),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=60,
    )
    records = [json.loads(line) for line in completed.stdout.splitlines() if line.strip()]
    assert len(records) == PROBE_CYCLES + 1
    _annotate_service_actions(records, monkeypatch)
    _write_artifact(records, lateral_sign, strict_lane_hint)

    candidate = records[0]
    cycles = records[1:]

    assert candidate["record_type"] == "candidate"
    assert candidate["strict_lane_hint"] is strict_lane_hint
    assert candidate["requested_lateral_sign"] == pytest.approx(lateral_sign)
    assert candidate["pure_vehicle_id"] == candidate["feedback_vehicle_id"]
    assert candidate["pure_simulation_time"] == pytest.approx(
        candidate["feedback_simulation_time"], abs=1e-12
    )
    assert candidate["pure_position"] == candidate["feedback_position"]
    assert candidate["pure_state"] == candidate["feedback_state"]

    candidate_state = candidate["pure_state"]
    assert candidate_state["speed"] <= 0.2
    assert lateral_sign * candidate_state["lateral_speed"] >= 0.5
    assert candidate_state["service_lane_change_intent"] == "none"
    assert candidate_state["lca_bit_intent"] == "none"

    required_snapshot_fields = {
        "primary_lane",
        "target_lane",
        "shadow_lane",
        "lca_bit_intent",
        "service_lane_change_intent",
        "service_lane_change_target_lane_id",
        "speed",
        "lateral_speed",
        "reported_lateral_speed",
        "lateral_position",
        "lane_position",
        "lane_index",
        "lane_angle",
        "angle",
        "angle_offset_degrees",
        "completion",
        "direction",
        "changing",
        "maneuver_distance",
        "previous_maneuver_distance",
        "own_state",
        "left_state",
        "right_state",
    }
    for record in cycles:
        assert record["record_type"] == "cycle"
        assert record["strict_lane_hint"] is strict_lane_hint
        assert record["pure_phase_b_time"] == pytest.approx(record["phase_b_time"], abs=1e-9)
        assert record["phase_a_observed_x"] == pytest.approx(
            record["phase_a_requested_x"], abs=POSITION_TOLERANCE
        )
        assert record["phase_a_observed_y"] == pytest.approx(
            record["phase_a_requested_y"], abs=POSITION_TOLERANCE
        )
        assert (
            _angle_difference(
                record["phase_a_observed_angle"],
                record["phase_a_requested_angle"],
            )
            < ANGLE_TOLERANCE
        )
        for branch in ("pre_phase_a", "phase_a", "phase_b", "pure"):
            snapshot = record[branch]
            assert required_snapshot_fields <= snapshot.keys()
            assert snapshot["service_lane_change_intent"] == snapshot["lca_bit_intent"]

    assert cycles[0]["pre_phase_a"] == candidate["feedback_state"]
    same_primary_updates = []
    active_maneuver_updates = []
    for previous, current in zip(cycles, cycles[1:]):
        assert current["pre_phase_a"] == previous["phase_b"]
        previous_phase_a = previous["phase_a"]
        pre_phase_a = current["pre_phase_a"]
        phase_a = current["phase_a"]
        if not (
            previous_phase_a["primary_lane"]
            == pre_phase_a["primary_lane"]
            == phase_a["primary_lane"]
        ):
            continue

        same_primary_updates.append(current)
        assert phase_a["lateral_speed"] == pytest.approx(0.0, abs=1e-9)
        assert phase_a["reported_lateral_speed"] == pytest.approx(0.0, abs=1e-9)

        if abs(pre_phase_a["maneuver_distance"]) < POSITION_TOLERANCE:
            assert phase_a["maneuver_distance"] == pytest.approx(
                0.0, abs=POSITION_TOLERANCE
            )
            continue
        if (
            abs(previous_phase_a["maneuver_distance"]) >= POSITION_TOLERANCE
            and previous_phase_a["direction"]
            == pre_phase_a["direction"]
            == phase_a["direction"]
            and previous_phase_a["target_lane"]
            == pre_phase_a["target_lane"]
            == phase_a["target_lane"]
        ):
            active_maneuver_updates.append(current)
            assert phase_a["maneuver_distance"] == pytest.approx(
                previous_phase_a["maneuver_distance"], abs=POSITION_TOLERANCE
            )

    assert same_primary_updates
    assert active_maneuver_updates
    rebased_angle_offsets = [
        record["phase_a"]["angle_offset_degrees"]
        for record in same_primary_updates
    ]
    assert max(abs(offset) for offset in rebased_angle_offsets) < 5.0
    assert max(rebased_angle_offsets) - min(rebased_angle_offsets) < ANGLE_TOLERANCE

    stable_primary_updates = [
        record
        for record in same_primary_updates
        if record["phase_a"]["primary_lane"] == record["phase_b"]["primary_lane"]
    ]
    assert stable_primary_updates
    assert max(
        record["phase_a_b_angle_delta"] for record in stable_primary_updates
    ) < 45.0
    assert any(
        abs(record["phase_b"]["lateral_speed"]) > 1e-3
        for record in active_maneuver_updates
    )
