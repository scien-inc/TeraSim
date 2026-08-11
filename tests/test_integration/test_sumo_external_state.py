"""One-vehicle gate for the dedicated SUMO setExternalState build."""

import math
import os
import shutil
import subprocess
import textwrap
from pathlib import Path
from types import SimpleNamespace

import pytest

STEP_LENGTH = 0.05
POSITION_TOLERANCE = 1e-6
ANGLE_TOLERANCE = 1e-6
SPEED_TOLERANCE = 1e-6


def _find_binary(name: str) -> str:
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        candidate = Path(sumo_home) / "bin" / name
        if candidate.is_file():
            return str(candidate)
    candidate = shutil.which(name)
    if candidate:
        return candidate
    pytest.skip(f"{name} binary is not available")


def _angle_difference(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)


def _signed_lateral_offset(position, lane_start, direction) -> float:
    left_normal = (-direction[1], direction[0])
    return (position[0] - lane_start[0]) * left_normal[0] + (
        position[1] - lane_start[1]
    ) * left_normal[1]


def _write_straight_network(
    tmp_path: Path, netconvert: str, *, lefthand: bool = False
) -> tuple[Path, Path]:
    nodes_path = tmp_path / "straight.nod.xml"
    edges_path = tmp_path / "straight.edg.xml"
    network_path = tmp_path / "straight.net.xml"
    routes_path = tmp_path / "straight.rou.xml"

    nodes_path.write_text(
        textwrap.dedent(
            """\
            <nodes>
                <node id="start" x="0" y="0" type="dead_end"/>
                <node id="end" x="500" y="0" type="dead_end"/>
            </nodes>
            """
        ),
        encoding="utf-8",
    )
    edges_path.write_text(
        textwrap.dedent(
            """\
            <edges>
                <edge id="road" from="start" to="end" numLanes="1" speed="30"/>
            </edges>
            """
        ),
        encoding="utf-8",
    )
    routes_path.write_text(
        textwrap.dedent(
            """\
            <routes>
                <vType id="car" accel="2.6" decel="4.5" sigma="0"
                       length="5" maxSpeed="30"/>
                <route id="route" edges="road"/>
                <vehicle id="ego" type="car" route="route" depart="0"
                         departPos="20" departSpeed="10"/>
            </routes>
            """
        ),
        encoding="utf-8",
    )
    subprocess.run(
        [
            netconvert,
            "--node-files",
            str(nodes_path),
            "--edge-files",
            str(edges_path),
            "--output-file",
            str(network_path),
            "--lefthand",
            str(lefthand).lower(),
            "--no-warnings",
            "true",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return network_path, routes_path


def _position_at_declared_lane_offset(shape, declared_length: float, offset: float):
    """Interpolate shape geometry using SUMO's declared lane-length coordinate."""
    shape_length = sum(math.dist(start, end) for start, end in zip(shape, shape[1:]))
    geometry_offset = min(max(offset / declared_length, 0.0), 1.0) * shape_length
    traversed = 0.0
    for start, end in zip(shape, shape[1:]):
        segment_length = math.dist(start, end)
        if traversed + segment_length >= geometry_offset:
            ratio = (geometry_offset - traversed) / segment_length
            return (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
        traversed += segment_length
    return shape[-1]


def _write_odaiba_edge426_route(tmp_path: Path) -> tuple[Path, Path]:
    network_path = (
        Path(__file__).resolve().parents[2]
        / "examples/maps/odaiba_ll2/tlmappings_0708/network.net.xml"
    )
    if not network_path.is_file():
        pytest.skip("Odaiba network is not available")
    routes_path = tmp_path / "edge426.rou.xml"
    routes_path.write_text(
        textwrap.dedent(
            """\
            <routes>
                <vType id="car" accel="2.6" decel="4.5" emergencyDecel="9"
                       sigma="0" length="5" width="1.8" maxSpeed="16.667"
                       laneChangeModel="SL2015"/>
                <route id="route"
                       edges="edge_426 edge_432 edge_427 edge_52 edge_54 edge_59 edge_255"/>
                <vehicle id="ego" type="car" route="route" depart="0"
                         departLane="1" departPos="45" departSpeed="9"/>
            </routes>
            """
        ),
        encoding="utf-8",
    )
    return network_path, routes_path


@pytest.mark.integration
@pytest.mark.requires_sumo
def test_external_state_assimilation_and_single_step_progression(tmp_path: Path) -> None:
    """Phase A is immediate and Phase B is one normal 0.05-second step."""
    traci = pytest.importorskip("traci")
    if not hasattr(traci.vehicle, "setExternalState"):
        pytest.skip("requires the dedicated SUMO setExternalState build")

    assert traci.constants.VAR_EXTERNAL_STATE == 0xF8
    sumo = _find_binary("sumo")
    netconvert = _find_binary("netconvert")
    network_path, routes_path = _write_straight_network(tmp_path, netconvert)

    traci.start(
        [
            sumo,
            "--net-file",
            str(network_path),
            "--route-files",
            str(routes_path),
            "--step-length",
            str(STEP_LENGTH),
            "--no-step-log",
            "true",
            "--duration-log.disable",
            "true",
        ],
        numRetries=5,
    )
    try:
        traci.simulationStep()
        assert traci.vehicle.getIDList() == ("ego",)

        lane_shape = traci.lane.getShape("road_0")
        start, end = lane_shape[0], lane_shape[-1]
        lane_length = math.dist(start, end)
        direction = ((end[0] - start[0]) / lane_length, (end[1] - start[1]) / lane_length)
        target_position = (
            start[0] + direction[0] * 40.0,
            start[1] + direction[1] * 40.0,
        )
        target_angle = traci.vehicle.getAngle("ego")
        target_speed = 10.0
        previous_phase_b = None
        observed_speeds = []
        observed_angles = []

        for _cycle in range(12):
            if previous_phase_b is not None:
                previous_position, previous_angle, previous_speed = previous_phase_b
                target_position = (
                    previous_position[0] + direction[0] * 0.01,
                    previous_position[1] + direction[1] * 0.01,
                )
                target_angle = previous_angle
                target_speed = previous_speed
                assert math.dist(target_position, previous_position) < 0.02

            phase_a_time = traci.simulation.getTime()
            traci.vehicle.setExternalState(
                "ego",
                "road",
                0,
                target_position[0],
                target_position[1],
                target_angle,
                target_speed,
                0.0,
                keepRoute=1,
                matchThreshold=10.0,
            )

            phase_a_position = traci.vehicle.getPosition("ego")
            phase_a_angle = traci.vehicle.getAngle("ego")
            phase_a_speed = traci.vehicle.getSpeed("ego")
            phase_a_lane_position = traci.vehicle.getLanePosition("ego")

            assert traci.simulation.getTime() == phase_a_time
            assert math.dist(phase_a_position, target_position) < POSITION_TOLERANCE
            assert _angle_difference(phase_a_angle, target_angle) < ANGLE_TOLERANCE
            assert abs(phase_a_speed - target_speed) < SPEED_TOLERANCE

            traci.simulationStep()

            phase_b_time = traci.simulation.getTime()
            phase_b_position = traci.vehicle.getPosition("ego")
            phase_b_angle = traci.vehicle.getAngle("ego")
            phase_b_speed = traci.vehicle.getSpeed("ego")
            phase_b_lane_position = traci.vehicle.getLanePosition("ego")

            assert phase_b_time == pytest.approx(phase_a_time + STEP_LENGTH)
            assert traci.vehicle.getLaneID("ego") == "road_0"
            assert phase_b_lane_position > phase_a_lane_position + 0.1
            assert math.dist(phase_b_position, phase_a_position) > 0.1
            assert abs(phase_b_speed - phase_a_speed) < 1.0
            assert _angle_difference(phase_b_angle, phase_a_angle) < 5.0

            observed_speeds.extend((phase_a_speed, phase_b_speed))
            observed_angles.extend((phase_a_angle, phase_b_angle))
            previous_phase_b = (phase_b_position, phase_b_angle, phase_b_speed)

        speed_jumps = [
            abs(right - left) for left, right in zip(observed_speeds, observed_speeds[1:])
        ]
        angle_jumps = [
            _angle_difference(right, left)
            for left, right in zip(observed_angles, observed_angles[1:])
        ]
        assert max(speed_jumps) < 1.0
        assert max(angle_jumps) < 5.0
    finally:
        traci.close()


@pytest.mark.integration
@pytest.mark.requires_sumo
def test_external_state_releases_stale_traci_speed_latch(tmp_path: Path) -> None:
    """Phase B must start from the assimilated speed, not an older setSpeed."""
    traci = pytest.importorskip("traci")
    if not hasattr(traci.vehicle, "setExternalState"):
        pytest.skip("requires the dedicated SUMO setExternalState build")

    sumo = _find_binary("sumo")
    netconvert = _find_binary("netconvert")
    network_path, routes_path = _write_straight_network(tmp_path, netconvert)
    traci.start(
        [
            sumo,
            "--net-file",
            str(network_path),
            "--route-files",
            str(routes_path),
            "--step-length",
            str(STEP_LENGTH),
            "--no-step-log",
            "true",
            "--duration-log.disable",
            "true",
        ],
        numRetries=5,
    )
    try:
        traci.simulationStep()
        assert traci.vehicle.getIDList() == ("ego",)

        # This mirrors NADEWithAV.add_av_unsafe(): setSpeed creates an advice
        # timeline that otherwise remains active until explicitly released.
        stale_speed = 16.845
        traci.vehicle.setSpeedMode("ego", 0)
        traci.vehicle.setSpeed("ego", stale_speed)

        lane_shape = traci.lane.getShape("road_0")
        start, end = lane_shape[0], lane_shape[-1]
        lane_length = math.dist(start, end)
        direction = (
            (end[0] - start[0]) / lane_length,
            (end[1] - start[1]) / lane_length,
        )
        target_position = (
            start[0] + direction[0] * 40.0,
            start[1] + direction[1] * 40.0,
        )
        target_angle = traci.vehicle.getAngle("ego")
        target_speed = 0.065
        phase_a_time = traci.simulation.getTime()

        traci.vehicle.setExternalState(
            "ego",
            "road",
            0,
            target_position[0],
            target_position[1],
            target_angle,
            target_speed,
            0.0,
            keepRoute=1,
            matchThreshold=10.0,
        )

        phase_a_position = traci.vehicle.getPosition("ego")
        assert traci.simulation.getTime() == phase_a_time
        assert math.dist(phase_a_position, target_position) < POSITION_TOLERANCE
        assert traci.vehicle.getSpeed("ego") == pytest.approx(
            target_speed, abs=SPEED_TOLERANCE
        )

        traci.simulationStep()

        phase_b_position = traci.vehicle.getPosition("ego")
        phase_b_displacement = math.dist(phase_b_position, phase_a_position)
        assert traci.simulation.getTime() == pytest.approx(phase_a_time + STEP_LENGTH)
        assert phase_b_displacement < 0.05
        assert traci.vehicle.getSpeed("ego") < 1.0
    finally:
        traci.close()


@pytest.mark.integration
@pytest.mark.requires_sumo
def test_external_state_preserves_off_center_side_on_left_hand_network(
    tmp_path: Path,
) -> None:
    """Releasing the remote latch must not mirror the lateral position."""
    traci = pytest.importorskip("traci")
    if not hasattr(traci.vehicle, "setExternalState"):
        pytest.skip("requires the dedicated SUMO setExternalState build")

    sumo = _find_binary("sumo")
    netconvert = _find_binary("netconvert")
    network_path, routes_path = _write_straight_network(
        tmp_path, netconvert, lefthand=True
    )
    traci.start(
        [
            sumo,
            "--net-file",
            str(network_path),
            "--route-files",
            str(routes_path),
            "--step-length",
            str(STEP_LENGTH),
            "--lateral-resolution",
            "0.2",
            "--no-step-log",
            "true",
            "--duration-log.disable",
            "true",
        ],
        numRetries=5,
    )
    try:
        traci.simulationStep()
        lane_start, lane_end = traci.lane.getShape("road_0")
        lane_length = math.dist(lane_start, lane_end)
        direction = (
            (lane_end[0] - lane_start[0]) / lane_length,
            (lane_end[1] - lane_start[1]) / lane_length,
        )
        left_normal = (-direction[1], direction[0])
        target = (
            lane_start[0] + direction[0] * 40.0 + left_normal[0] * 0.4,
            lane_start[1] + direction[1] * 40.0 + left_normal[1] * 0.4,
        )
        target_angle = traci.vehicle.getAngle("ego")
        target_speed = 0.065
        observed_lateral_offsets = []

        for _cycle in range(6):
            target_lateral = _signed_lateral_offset(target, lane_start, direction)
            phase_a_time = traci.simulation.getTime()
            traci.vehicle.setExternalState(
                "ego",
                "road",
                0,
                target[0],
                target[1],
                target_angle,
                target_speed,
                0.0,
                keepRoute=1,
                matchThreshold=10.0,
            )
            phase_a_position = traci.vehicle.getPosition("ego")
            phase_a_lateral = _signed_lateral_offset(
                phase_a_position, lane_start, direction
            )
            assert traci.simulation.getTime() == phase_a_time
            assert math.dist(phase_a_position, target) < POSITION_TOLERANCE
            assert phase_a_lateral == pytest.approx(target_lateral, abs=POSITION_TOLERANCE)
            assert traci.vehicle.getLateralLanePosition("ego") < -0.3

            traci.simulationStep()

            phase_b_position = traci.vehicle.getPosition("ego")
            phase_b_lateral = _signed_lateral_offset(
                phase_b_position, lane_start, direction
            )
            assert traci.simulation.getTime() == pytest.approx(
                phase_a_time + STEP_LENGTH
            )
            assert phase_b_lateral > 0.2
            assert abs(phase_b_lateral - phase_a_lateral) < 0.08
            assert math.dist(phase_b_position, phase_a_position) < 0.08
            observed_lateral_offsets.extend((phase_a_lateral, phase_b_lateral))

            target = phase_b_position
            target_angle = traci.vehicle.getAngle("ego")
            target_speed = traci.vehicle.getSpeed("ego")

        assert max(observed_lateral_offsets) - min(observed_lateral_offsets) < 0.15
    finally:
        traci.close()


@pytest.mark.integration
@pytest.mark.requires_sumo
def test_external_state_edge426_lane_boundary_has_no_phase_b_warp(
    tmp_path: Path,
) -> None:
    """A left-hand primary-lane switch must preserve the assimilated x/y state."""
    traci = pytest.importorskip("traci")
    if not hasattr(traci.vehicle, "setExternalState"):
        pytest.skip("requires the dedicated SUMO setExternalState build")

    sumo = _find_binary("sumo")
    network_path, routes_path = _write_odaiba_edge426_route(tmp_path)
    traci.start(
        [
            sumo,
            "--net-file",
            str(network_path),
            "--route-files",
            str(routes_path),
            "--step-length",
            str(STEP_LENGTH),
            "--lateral-resolution",
            "0.2",
            "--no-step-log",
            "true",
            "--duration-log.disable",
            "true",
        ],
        numRetries=5,
    )
    try:
        traci.simulationStep()
        assert traci.vehicle.getIDList() == ("ego",)

        lane_2_shape = traci.lane.getShape("edge_426_2")
        lane_1_shape = traci.lane.getShape("edge_426_1")
        lane_2_length = traci.lane.getLength("edge_426_2")
        lane_1_length = traci.lane.getLength("edge_426_1")
        previous_phase_b_position = None
        phase_a_corrections = []
        phase_b_displacements = []
        phase_b_lanes = []

        for cycle in range(80):
            lane_offset = 45.0 + (cycle + 1) * 9.0 * STEP_LENGTH
            lane_2_center = _position_at_declared_lane_offset(
                lane_2_shape, lane_2_length, lane_offset
            )
            lane_1_center = _position_at_declared_lane_offset(
                lane_1_shape, lane_1_length, lane_offset
            )
            # Move continuously from lane 1 toward lane 2. The lane hint changes
            # only after the physical pose passes the geometric midpoint.
            lane_2_fraction = 0.10 + 0.50 * cycle / 79.0
            target = (
                lane_1_center[0]
                + lane_2_fraction * (lane_2_center[0] - lane_1_center[0]),
                lane_1_center[1]
                + lane_2_fraction * (lane_2_center[1] - lane_1_center[1]),
            )
            next_lane_1_center = _position_at_declared_lane_offset(
                lane_1_shape, lane_1_length, lane_offset + 0.1
            )
            target_angle = math.degrees(
                math.atan2(
                    next_lane_1_center[0] - lane_1_center[0],
                    next_lane_1_center[1] - lane_1_center[1],
                )
            ) % 360.0
            lane_hint = 1 if lane_2_fraction <= 0.5 else 2

            if previous_phase_b_position is not None:
                phase_a_corrections.append(
                    math.dist(previous_phase_b_position, target)
                )
            phase_a_time = traci.simulation.getTime()
            traci.vehicle.setExternalState(
                "ego",
                "edge_426",
                lane_hint,
                target[0],
                target[1],
                target_angle,
                9.0,
                0.0,
                keepRoute=1,
                matchThreshold=10.0,
            )
            assert traci.simulation.getTime() == phase_a_time
            assert math.dist(traci.vehicle.getPosition("ego"), target) < POSITION_TOLERANCE

            # Reproduce the stale opposite-direction maneuver seen in the field:
            # the external actor moves toward lane 2 while SUMO still carries a
            # lane-0 request from the preceding planning state.
            traci.vehicle.changeLane("ego", 0, 10.0)
            traci.simulation.executeMove()
            assert traci.simulation.getTime() == phase_a_time
            traci.simulationStep()

            phase_b_position = traci.vehicle.getPosition("ego")
            phase_b_displacements.append(math.dist(target, phase_b_position))
            phase_b_lanes.append(traci.vehicle.getLaneID("ego"))
            previous_phase_b_position = phase_b_position

        assert max(phase_b_displacements) < 1.2
        assert max(phase_a_corrections) < 1.2
        assert all(lane in {"edge_426_1", "edge_426_2"} for lane in phase_b_lanes)
    finally:
        traci.close()


def _new_external_state_cosim_plugin(plugin_module, simulator):
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.simulator = simulator
    plugin.controlled_agents_each_step = set()
    plugin.feedback_observed_speeds = {}
    plugin.feedback_observed_positions = {}
    plugin.feedback_observed_rear_axle_positions = {}
    plugin.feedback_observed_lane_progress = {}
    plugin.feedback_source_carla_frames = {}
    plugin.feedback_lane_change_active_actor_ids = set()
    plugin.feedback_lane_geometry_cache = {}
    plugin.feedback_edge_lane_ids_cache = {}
    plugin.feedback_lane_states = {}
    plugin.ackermann_feedback_lane_change_settings_applied = set()
    plugin.ackermann_feedback_lc_keep_right = None
    plugin.ackermann_feedback_assimilation_mode = "external_state"
    plugin.ackermann_feedback_validate_external_state = True
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_max_elevation_error = 2.0
    plugin.ackermann_feedback_signal_stop_line_clamp_offset = 1.01
    plugin.ackermann_feedback_log_lane_transitions = False
    plugin.continue_on_ackermann_feedback_failure = False
    plugin.continue_on_background_ackermann_feedback_failure = False
    plugin.logger = SimpleNamespace(
        info=lambda *args, **kwargs: None,
        debug=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )
    return plugin


@pytest.mark.integration
@pytest.mark.requires_sumo
def test_terasim_pipeline_assimilates_then_plans_then_steps_once(
    tmp_path: Path, monkeypatch
) -> None:
    """The real TeraSim priorities execute Phase A, planning, then one step."""
    traci = pytest.importorskip("traci")
    if not hasattr(traci.vehicle, "setExternalState"):
        pytest.skip("requires the dedicated SUMO setExternalState build")

    from terasim import simulator as simulator_module
    from terasim.pipeline import Pipeline, PipelineElement
    from terasim.simulator import Simulator
    from terasim_service.plugins import cosim as plugin_module

    monkeypatch.setattr(plugin_module, "traci", traci)
    monkeypatch.setattr(simulator_module, "traci", traci)

    sumo = _find_binary("sumo")
    netconvert = _find_binary("netconvert")
    network_path, routes_path = _write_straight_network(tmp_path, netconvert)
    traci.start(
        [
            sumo,
            "--net-file",
            str(network_path),
            "--route-files",
            str(routes_path),
            "--step-length",
            str(STEP_LENGTH),
            "--no-step-log",
            "true",
            "--duration-log.disable",
            "true",
        ],
        numRetries=5,
    )
    try:
        traci.simulationStep()
        assert traci.vehicle.getIDList() == ("ego",)

        simulator = Simulator.__new__(Simulator)
        simulator.sumo_net = None
        plugin = _new_external_state_cosim_plugin(plugin_module, simulator)

        lane_shape = traci.lane.getShape("road_0")
        start, end = lane_shape[0], lane_shape[-1]
        lane_length = math.dist(start, end)
        direction = (
            (end[0] - start[0]) / lane_length,
            (end[1] - start[1]) / lane_length,
        )
        target = {
            "position": (
                start[0] + direction[0] * 40.0,
                start[1] + direction[1] * 40.0,
            ),
            "angle": traci.vehicle.getAngle("ego"),
            "speed": 10.0,
        }
        events = []
        phase_a_states = []
        phase_b_states = []

        def assimilate(_simulator, _ctx):
            plugin.controlled_agents_each_step.clear()
            before_time = traci.simulation.getTime()
            command = {
                "agent_id": "ego",
                "agent_type": "vehicle",
                "command_type": "set_state",
                "data": {
                    "position": list(target["position"]),
                    "sumo_angle": target["angle"],
                    "speed": target["speed"],
                    "acceleration": 0.0,
                    "source_carla_frame": len(phase_a_states) + 1,
                },
            }
            assert plugin._handle_agent_command(command) is True
            state = (
                traci.simulation.getTime(),
                traci.vehicle.getPosition("ego"),
                traci.vehicle.getAngle("ego"),
                traci.vehicle.getSpeed("ego"),
                traci.vehicle.getLanePosition("ego"),
            )
            assert state[0] == before_time
            assert math.dist(state[1], target["position"]) < POSITION_TOLERANCE
            assert _angle_difference(state[2], target["angle"]) < ANGLE_TOLERANCE
            assert abs(state[3] - target["speed"]) < SPEED_TOLERANCE
            phase_a_states.append(state)
            events.append("phase_a")
            return True

        def decide_and_prepare(_simulator, _ctx):
            # This is the priority-0 env slot used by TeraSim/NADE. It observes
            # the already assimilated state and runs SUMO's planning phase.
            assert traci.vehicle.getPosition("ego") == phase_a_states[-1][1]
            traci.vehicle.setSpeed("ego", target["speed"])
            traci.simulation.executeMove()
            events.append("decision_execute_move")
            return True

        def step_once(_simulator, ctx):
            events.append("simulation_step")
            return simulator.sumo_step(simulator, ctx)

        simulator.step_pipeline = Pipeline(
            "external_state_phase_pipeline",
            [
                PipelineElement("phase_a_assimilation", assimilate, priority=-90),
                PipelineElement("terasim_nade_decision", decide_and_prepare, priority=0),
                PipelineElement("sumo_step", step_once, priority=10),
            ],
        )
        ctx = {}
        for _cycle in range(12):
            phase_a_time = traci.simulation.getTime()
            assert simulator.step_pipeline(simulator, ctx) is True
            phase_b_state = (
                traci.simulation.getTime(),
                traci.vehicle.getPosition("ego"),
                traci.vehicle.getAngle("ego"),
                traci.vehicle.getSpeed("ego"),
                traci.vehicle.getLanePosition("ego"),
            )
            assert phase_b_state[0] == pytest.approx(phase_a_time + STEP_LENGTH)
            assert phase_b_state[4] > phase_a_states[-1][4] + 0.1
            phase_b_states.append(phase_b_state)
            target = {
                "position": (
                    phase_b_state[1][0] + direction[0] * 0.01,
                    phase_b_state[1][1] + direction[1] * 0.01,
                ),
                "angle": phase_b_state[2],
                "speed": phase_b_state[3],
            }
            assert math.dist(target["position"], phase_b_state[1]) < 0.02

        assert events == [
            event
            for _cycle in range(12)
            for event in ("phase_a", "decision_execute_move", "simulation_step")
        ]
        observed_speeds = [
            state[3] for pair in zip(phase_a_states, phase_b_states) for state in pair
        ]
        observed_angles = [
            state[2] for pair in zip(phase_a_states, phase_b_states) for state in pair
        ]
        assert (
            max(abs(right - left) for left, right in zip(observed_speeds, observed_speeds[1:]))
            < 1.0
        )
        assert (
            max(
                _angle_difference(right, left)
                for left, right in zip(observed_angles, observed_angles[1:])
            )
            < 5.0
        )
    finally:
        traci.close()
