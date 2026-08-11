import json
import math
import sys
import types

import pytest


class FakeLocation:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeRotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class FakeTransform:
    def __init__(self, location=None, rotation=None):
        self.location = location or FakeLocation()
        self.rotation = rotation or FakeRotation()


class FakeVector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeCommand:
    FutureActor = object()

    @staticmethod
    def ApplyTransform(actor_id, transform):
        return ("transform", actor_id, transform)

    @staticmethod
    def ApplyVehicleAckermannControl(actor_id, control):
        return ("ackermann", actor_id, control)

    @staticmethod
    def ApplyVehicleControl(actor_id, control):
        return ("vehicle_control", actor_id, control)

    @staticmethod
    def SpawnActor(blueprint, transform):
        return types.SimpleNamespace(then=lambda command: ("spawn", blueprint, transform, command))

    @staticmethod
    def SetSimulatePhysics(actor_id, enabled):
        return ("simulate_physics", actor_id, enabled)

    @staticmethod
    def DestroyActor(actor_id):
        return ("destroy", actor_id)


class FakeVehicleAckermannControl:
    def __init__(self, steer=0.0, speed=0.0, acceleration=0.0, jerk=0.0):
        self.steer = steer
        self.speed = speed
        self.acceleration = acceleration
        self.jerk = jerk


class FakeVehicleControl:
    def __init__(
        self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False
    ):
        self.throttle = throttle
        self.steer = steer
        self.brake = brake
        self.hand_brake = hand_brake
        self.reverse = reverse


class FakeAckermannControllerSettings:
    def __init__(self, speed_kp, speed_ki, speed_kd, accel_kp, accel_ki, accel_kd):
        self.speed_kp = speed_kp
        self.speed_ki = speed_ki
        self.speed_kd = speed_kd
        self.accel_kp = accel_kp
        self.accel_ki = accel_ki
        self.accel_kd = accel_kd


def install_fake_carla():
    fake_carla = types.SimpleNamespace(
        Location=FakeLocation,
        Rotation=FakeRotation,
        Transform=FakeTransform,
        Vector3D=FakeVector3D,
        VehicleAckermannControl=FakeVehicleAckermannControl,
        VehicleControl=FakeVehicleControl,
        AckermannControllerSettings=FakeAckermannControllerSettings,
        command=FakeCommand,
    )
    sys.modules["carla"] = fake_carla
    imported_cosim = sys.modules.get("terasim_service.utils.carla.cosim")
    if imported_cosim is not None:
        imported_cosim.carla = fake_carla


def test_create_simulator_step_length_override_reaches_sumo(monkeypatch):
    from terasim_service.utils import base as base_module

    created = []

    class FakeSimulator:
        def __init__(self, **kwargs):
            created.append(kwargs)

    monkeypatch.setattr(base_module, "Simulator", FakeSimulator)
    config = {
        "input": {
            "sumo_net_file": "network.net.xml",
            "sumo_config_file": "scenario.sumocfg",
        },
        "simulator": {
            "parameters": {
                "num_tries": 1,
                "gui_flag": False,
                "realtime_flag": False,
                "sumo_output_file_types": [],
                "traffic_scale": 1.0,
                "step_length": 0.1,
            }
        },
    }

    base_module.create_simulator(config, "/tmp/output", step_length=0.05)
    base_module.create_simulator(config, "/tmp/output")

    assert created[0]["step_length"] == pytest.approx(0.05)
    assert created[1]["step_length"] == pytest.approx(0.1)


def test_lane_lookahead_crosses_internal_and_destination_lanes():
    from terasim_service.utils.sumo_lane_geometry import (
        extract_next_link_lane_ids,
        find_lookahead_position_from_lane_shapes,
    )

    next_links = [
        ("outgoing_0", ":junction_0_0", True, True, False, "G", "s", 12.5),
    ]
    assert extract_next_link_lane_ids(next_links) == [":junction_0_0", "outgoing_0"]
    sumo_123_links = [
        ("outgoing_0", True, True, False, ":junction_0_0", "G", "s", 12.5),
    ]
    assert extract_next_link_lane_ids(sumo_123_links) == [
        ":junction_0_0",
        "outgoing_0",
    ]
    point = find_lookahead_position_from_lane_shapes(
        [[(0.0, 0.0), (10.0, 0.0)], [(10.0, 0.0), (20.0, 0.0)]],
        (8.0, 0.0),
        7.0,
        1.5,
    )
    assert point == pytest.approx((15.0, 0.0, 1.5))


def test_lane_relative_position_preserves_longitudinal_and_lateral_offsets():
    from terasim_service.utils.sumo_lane_geometry import (
        reconstruct_position_from_lane_geometry,
    )

    reconstructed = reconstruct_position_from_lane_geometry(
        [(0.0, 0.0), (20.0, 0.0)],
        lane_position=7.5,
        lateral_offset=1.25,
        z=2.0,
        lane_length=20.0,
    )

    assert reconstructed == pytest.approx((7.5, 1.25, 2.0))


def test_lane_relative_position_uses_raw_position_to_disambiguate_left_hand_sign():
    from terasim_service.utils.sumo_lane_geometry import (
        reconstruct_position_from_lane_geometry,
    )

    reconstructed = reconstruct_position_from_lane_geometry(
        [(0.0, 0.0), (20.0, 0.0)],
        lane_position=7.5,
        lateral_offset=-1.25,
        z=2.0,
        lane_length=20.0,
        reference_position=(7.5, 1.25),
    )

    assert reconstructed == pytest.approx((7.5, 1.25, 2.0))


def test_lane_relative_position_uses_normalized_declared_lane_progress():
    from terasim_service.utils.sumo_lane_geometry import (
        reconstruct_position_from_lane_geometry,
    )

    reconstructed = reconstruct_position_from_lane_geometry(
        [(0.0, 0.0), (90.0, 0.0)],
        lane_position=50.0,
        lateral_offset=-1.0,
        z=0.0,
        lane_length=100.0,
    )

    assert reconstructed == pytest.approx((45.0, -1.0, 0.0))


def test_carla_prefers_lane_relative_target_and_falls_back_to_raw_position():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._invalid_location_warnings = set()
    actor_info = {
        "x": 10.0,
        "y": 20.0,
        "z": 1.0,
        "reconstructed_x": 11.0,
        "reconstructed_y": 21.0,
        "reconstructed_z": 1.5,
        "reconstructed_position_valid": True,
    }

    assert cosim._resolve_sumo_location(
        "vehicle", "BV", actor_info, prefer_lane_relative=True
    ) == pytest.approx([11.0, 21.0, 1.5])
    assert cosim._resolve_sumo_location(
        "vehicle", "BV", actor_info, prefer_lane_relative=False
    ) == pytest.approx([10.0, 20.0, 1.0])

    actor_info["reconstructed_x"] = None
    assert cosim._resolve_sumo_location(
        "vehicle", "BV", actor_info, prefer_lane_relative=True
    ) == pytest.approx([10.0, 20.0, 1.0])


def test_batched_lane_lookahead_matches_scalar_results():
    from terasim_service.utils.sumo_lane_geometry import (
        compile_lane_shapes,
        find_lookahead_position_from_lane_shapes,
        find_lookahead_positions_from_compiled_paths,
    )

    lane_shapes = [
        [(0.0, 0.0), (10.0, 0.0)],
        [(10.0, 0.0), (10.0, 10.0)],
    ]
    compiled = compile_lane_shapes(lane_shapes)
    positions = [(8.0, 0.0), (10.0, 2.0), (10.0, 9.0)]
    distances = [5.0, 5.0, 5.0]
    z_values = [1.0, 2.0, 3.0]

    expected = [
        find_lookahead_position_from_lane_shapes(
            lane_shapes, position, distance, z
        )
        for position, distance, z in zip(positions, distances, z_values)
    ]
    actual = find_lookahead_positions_from_compiled_paths(
        [compiled, compiled, compiled], positions, distances, z_values
    )

    for actual_point, expected_point in zip(actual, expected):
        assert actual_point == pytest.approx(expected_point)


def test_vehicle_lookahead_projects_from_continuous_carla_feedback_position(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module
    from terasim_service.utils.messages.AgentStateSimplified import (
        AgentStateSimplified,
    )
    from terasim_service.utils.sumo_lane_geometry import compile_lane_shapes

    constants = types.SimpleNamespace(VAR_SPEED_LAT=1, VAR_LANEPOSITION_LAT=2)
    monkeypatch.setattr(
        plugin_module, "traci", types.SimpleNamespace(constants=constants)
    )
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_observed_positions = {"AV": (3.0, 0.0)}
    plugin.lookahead_straight_min_distance = 7.0
    plugin.lookahead_max_distance = 15.0
    plugin.lookahead_curve_min_distance = 3.5
    plugin.lookahead_curve_start_radians = math.radians(5.0)
    plugin.lookahead_curve_full_scale_radians = math.radians(45.0)
    plugin._get_vehicle_lookahead_compiled_path = lambda *args, **kwargs: (
        compile_lane_shapes([[(0.0, 0.0), (100.0, 0.0)]])
    )
    state = AgentStateSimplified(x=80.0, y=0.0, speed=0.0, sumo_angle=90.0)

    plugin._populate_vehicle_lookaheads(
        [(
            "AV",
            state,
            {constants.VAR_SPEED_LAT: 0.0, constants.VAR_LANEPOSITION_LAT: 0.0},
        )]
    )

    assert state.lookahead_origin_x == pytest.approx(3.0)
    assert state.lookahead_origin_y == pytest.approx(0.0)
    assert state.lookahead_x == pytest.approx(10.0)
    assert state.lookahead_y == pytest.approx(0.0)


def test_vehicle_lookahead_projects_from_carla_rear_axle_position(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module
    from terasim_service.utils.messages.AgentStateSimplified import (
        AgentStateSimplified,
    )
    from terasim_service.utils.sumo_lane_geometry import compile_lane_shapes

    constants = types.SimpleNamespace(VAR_SPEED_LAT=1, VAR_LANEPOSITION_LAT=2)
    monkeypatch.setattr(
        plugin_module, "traci", types.SimpleNamespace(constants=constants)
    )
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_observed_positions = {"AV": (3.0, 0.0)}
    plugin.feedback_observed_rear_axle_positions = {"AV": (1.0, 0.0)}
    plugin.lookahead_straight_min_distance = 7.0
    plugin.lookahead_max_distance = 15.0
    plugin.lookahead_curve_min_distance = 3.5
    plugin.lookahead_curve_start_radians = math.radians(5.0)
    plugin.lookahead_curve_full_scale_radians = math.radians(45.0)
    plugin._get_vehicle_lookahead_compiled_path = lambda *args, **kwargs: (
        compile_lane_shapes([[(0.0, 0.0), (100.0, 0.0)]])
    )
    state = AgentStateSimplified(x=80.0, y=0.0, speed=0.0, sumo_angle=90.0)

    plugin._populate_vehicle_lookaheads(
        [
            (
                "AV",
                state,
                {
                    constants.VAR_SPEED_LAT: 0.0,
                    constants.VAR_LANEPOSITION_LAT: 0.0,
                },
            )
        ]
    )

    assert state.lookahead_origin_x == pytest.approx(1.0)
    assert state.lookahead_origin_y == pytest.approx(0.0)
    assert state.lookahead_x == pytest.approx(8.0)
    assert state.lookahead_y == pytest.approx(0.0)


def test_vehicle_lookahead_exports_phase_aligned_lane_progress_error(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module
    from terasim_service.utils.messages.AgentStateSimplified import (
        AgentStateSimplified,
    )
    from terasim_service.utils.sumo_lane_geometry import compile_lane_shapes

    constants = types.SimpleNamespace(VAR_SPEED_LAT=1, VAR_LANEPOSITION_LAT=2)
    monkeypatch.setattr(
        plugin_module, "traci", types.SimpleNamespace(constants=constants)
    )
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_observed_positions = {"AV": (10.0, 0.0)}
    plugin.feedback_observed_rear_axle_positions = {"AV": (7.0, 0.0)}
    plugin.feedback_observed_speeds = {"AV": 10.0}
    plugin.feedback_observed_lane_progress = {"AV": ("edge_0_0", 10.0, 100.0)}
    plugin.ackermann_feedback_step_length = 0.05
    plugin.lookahead_straight_min_distance = 7.0
    plugin.lookahead_max_distance = 15.0
    plugin.lookahead_curve_min_distance = 3.5
    plugin.lookahead_curve_start_radians = math.radians(5.0)
    plugin.lookahead_curve_full_scale_radians = math.radians(45.0)
    plugin._get_vehicle_lookahead_compiled_path = lambda *args, **kwargs: (
        compile_lane_shapes([[(0.0, 0.0), (100.0, 0.0)]])
    )
    state = AgentStateSimplified(
        x=10.5,
        y=0.0,
        speed=10.0,
        sumo_angle=90.0,
        lane_id="edge_0_0",
        lane_position=10.5,
    )

    plugin._populate_vehicle_lookaheads(
        [
            (
                "AV",
                state,
                {
                    constants.VAR_SPEED_LAT: 0.0,
                    constants.VAR_LANEPOSITION_LAT: 0.0,
                },
            )
        ]
    )

    assert state.feedback_longitudinal_error == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("assimilation_mode", "expected_lookahead", "expected_blend"),
    [
        ("external_state", (10.0, 1.5), 0.0),
        ("legacy", (11.5, -0.9), 1.0),
    ],
)
def test_feedback_lookahead_blending_depends_on_assimilation_mode(
    monkeypatch,
    assimilation_mode,
    expected_lookahead,
    expected_blend,
):
    from terasim_service.plugins import cosim as plugin_module
    from terasim_service.utils.messages.AgentStateSimplified import (
        AgentStateSimplified,
    )
    from terasim_service.utils.sumo_lane_geometry import compile_lane_shapes

    constants = types.SimpleNamespace(VAR_SPEED_LAT=1, VAR_LANEPOSITION_LAT=2)
    monkeypatch.setattr(
        plugin_module, "traci", types.SimpleNamespace(constants=constants)
    )
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_observed_positions = {"AV": (8.0, -0.9)}
    plugin.feedback_source_carla_frames = {"AV": 123}
    plugin.ackermann_feedback_assimilation_mode = assimilation_mode
    plugin.external_state_route_lookahead_only = True
    plugin.lookahead_straight_min_distance = 7.0
    plugin.lookahead_max_distance = 15.0
    plugin.lookahead_curve_min_distance = 3.5
    plugin.lookahead_curve_start_radians = math.radians(5.0)
    plugin.lookahead_curve_full_scale_radians = math.radians(45.0)
    plugin._get_vehicle_lookahead_compiled_path = lambda *args, **kwargs: (
        compile_lane_shapes(
            [[(0.0, 0.0), (10.0, 0.0)], [(10.0, 0.0), (10.0, 20.0)]]
        )
    )
    state = AgentStateSimplified(
        x=8.0,
        y=-0.9,
        speed=10.0,
        sumo_angle=90.0,
        z=0.0,
    )

    plugin._populate_vehicle_lookaheads(
        [
            (
                "AV",
                state,
                {
                    constants.VAR_SPEED_LAT: 0.35,
                    constants.VAR_LANEPOSITION_LAT: -0.9,
                },
            )
        ]
    )

    assert state.lookahead_distance == pytest.approx(3.5)
    assert state.lookahead_heading_change == pytest.approx(math.pi / 2.0)
    assert state.lookahead_lane_change_blend == pytest.approx(expected_blend)
    assert state.lookahead_x == pytest.approx(expected_lookahead[0])
    assert state.lookahead_y == pytest.approx(expected_lookahead[1])


def test_external_state_lane_export_uses_live_state_over_stale_subscription(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module
    from terasim_service.utils.messages.AgentStateSimplified import (
        AgentStateSimplified,
    )

    constants = types.SimpleNamespace(
        VAR_LANE_ID=1,
        VAR_LANEPOSITION=2,
        VAR_LANEPOSITION_LAT=3,
    )
    calls = []
    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda actor_id: calls.append(("lane", actor_id)) or "road_0",
        getLanePosition=lambda actor_id: calls.append(("position", actor_id)) or 12.5,
        getLateralLanePosition=lambda actor_id: calls.append(("lateral", actor_id)) or 0.25,
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(constants=constants, vehicle=fake_vehicle),
    )
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.lane_relative_position_enabled = True
    plugin.ackermann_feedback_assimilation_mode = "external_state"
    plugin.feedback_observed_speeds = {"AV": 4.0}
    plugin._get_lookahead_lane_shape = lambda *_args, **_kwargs: [
        (0.0, 0.0),
        (100.0, 0.0),
    ]
    plugin._get_lane_length = lambda *_args, **_kwargs: 100.0
    state = AgentStateSimplified(x=12.5, y=0.25)

    plugin._populate_lane_relative_position(
        "AV",
        state,
        context_values={
            constants.VAR_LANE_ID: "",
            constants.VAR_LANEPOSITION: 0.0,
            constants.VAR_LANEPOSITION_LAT: 0.0,
        },
    )

    assert calls == [("lane", "AV"), ("position", "AV"), ("lateral", "AV")]
    assert state.lane_id == "road_0"
    assert state.lane_position == pytest.approx(12.5)
    assert state.lateral_offset == pytest.approx(0.25)
    assert state.reconstructed_x == pytest.approx(12.5)
    assert state.reconstructed_y == pytest.approx(0.25)


def test_lookahead_route_cache_avoids_repeated_traci_calls(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    constants = types.SimpleNamespace(VAR_LANE_ID=1, VAR_NEXT_LINKS=2, VAR_ROUTE_ID=3, VAR_EDGES=4)
    lane_shapes = {
        "edge_0_0": [(0.0, 0.0), (10.0, 0.0)],
        ":junction_0_0": [(10.0, 0.0), (12.0, 0.0)],
        "edge_1_0": [(12.0, 0.0), (30.0, 0.0)],
        ":junction_1_0": [(10.0, 0.0), (12.0, 4.0)],
        "edge_2_0": [(12.0, 4.0), (30.0, 4.0)],
    }
    shape_calls = []
    next_link_calls = []
    next_link_results = iter(
        [
            [("edge_1_0", ":junction_0_0", True, True, False, "G", "s", 2.0)],
            [("edge_2_0", ":junction_1_0", True, True, False, "G", "l", 2.0)],
        ]
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: shape_calls.append(lane_id) or lane_shapes[lane_id]
    )
    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _vehicle_id: (_ for _ in ()).throw(
            AssertionError("context lane ID must be reused")
        ),
        getNextLinks=lambda vehicle_id: next_link_calls.append(vehicle_id)
        or next(next_link_results),
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(
            constants=constants,
            lane=fake_lane,
            vehicle=fake_vehicle,
        ),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.lookahead_lane_shape_cache = {}
    plugin.lookahead_geometry_cache = {}
    context_values = {
        constants.VAR_LANE_ID: "edge_0_0",
        constants.VAR_ROUTE_ID: "route_a",
        constants.VAR_EDGES: ("edge_0", "edge_1"),
    }

    first = plugin._get_vehicle_lookahead_compiled_path(
        "AV", context_values=context_values
    )
    second = plugin._get_vehicle_lookahead_compiled_path(
        "AV", context_values=context_values
    )
    context_values[constants.VAR_EDGES] = ("edge_0", "edge_2")
    after_reroute = plugin._get_vehicle_lookahead_compiled_path(
        "AV", context_values=context_values
    )

    assert first is second
    assert first is not after_reroute
    assert next_link_calls == ["AV", "AV"]
    assert shape_calls == [
        "edge_0_0",
        ":junction_0_0",
        "edge_1_0",
        ":junction_1_0",
        "edge_2_0",
    ]
    assert len(plugin.lookahead_geometry_cache) == 2


def test_lookahead_route_without_route_signature_is_not_cached(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    constants = types.SimpleNamespace(
        VAR_LANE_ID=1, VAR_NEXT_LINKS=2, VAR_ROUTE_ID=3, VAR_EDGES=4
    )
    next_link_calls = []
    fake_vehicle = types.SimpleNamespace(
        getNextLinks=lambda vehicle_id: next_link_calls.append(vehicle_id) or []
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda _lane_id: [(0.0, 0.0), (10.0, 0.0)]
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(
            constants=constants, lane=fake_lane, vehicle=fake_vehicle
        ),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.lookahead_lane_shape_cache = {}
    plugin.lookahead_geometry_cache = {}
    context_values = {
        constants.VAR_LANE_ID: "edge_0_0",
        constants.VAR_ROUTE_ID: "route_a",
    }

    plugin._get_vehicle_lookahead_compiled_path("AV", context_values=context_values)
    plugin._get_vehicle_lookahead_compiled_path("AV", context_values=context_values)

    assert next_link_calls == ["AV", "AV"]


def test_lookahead_missing_lane_does_not_reuse_stale_path(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    constants = types.SimpleNamespace(
        VAR_LANE_ID=1, VAR_NEXT_LINKS=2, VAR_ROUTE_ID=3, VAR_EDGES=4
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(constants=constants),
    )
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)

    compiled = plugin._get_vehicle_lookahead_compiled_path(
        "AV", context_values={constants.VAR_LANE_ID: ""}
    )

    assert compiled is None


def test_route_aware_projection_switches_lane_after_centerline_midpoint():
    from terasim_service.utils.sumo_lane_geometry import (
        select_route_aware_lane_projection,
    )

    candidates = [
        {"lane_id": "edge_0_0", "shape": [(0.0, 0.0), (100.0, 0.0)], "length": 80.0},
        {"lane_id": "edge_0_1", "shape": [(0.0, 3.2), (100.0, 3.2)], "length": 80.0},
    ]
    source_projection = select_route_aware_lane_projection(
        (25.0, 1.6),
        90.0,
        candidates,
        current_lane_id="edge_0_0",
        lane_switch_hysteresis=0.35,
    )
    target_projection = select_route_aware_lane_projection(
        (25.0, 2.0),
        90.0,
        candidates,
        current_lane_id="edge_0_0",
        lane_switch_hysteresis=0.35,
    )

    assert source_projection["lane_id"] == "edge_0_0"
    assert target_projection["lane_id"] == "edge_0_1"
    assert target_projection["lane_position"] == pytest.approx(20.0)


def test_route_aware_projection_can_preserve_sumo_lane_change_state():
    from terasim_service.utils.sumo_lane_geometry import (
        select_route_aware_lane_projection,
    )

    candidates = [
        {"lane_id": "edge_0_0", "shape": [(0.0, 0.0), (100.0, 0.0)], "length": 100.0},
        {"lane_id": "edge_0_1", "shape": [(0.0, 3.2), (100.0, 3.2)], "length": 100.0},
    ]
    projection = select_route_aware_lane_projection(
        (25.0, 3.2), 90.0, candidates, current_lane_id="edge_0_0", prefer_current_lane=True
    )
    assert projection["lane_id"] == "edge_0_0"


def test_route_aware_projection_rejects_opposing_or_distant_lanes():
    from terasim_service.utils.sumo_lane_geometry import (
        select_route_aware_lane_projection,
    )

    candidates = [{"lane_id": "opposing_0", "shape": [(100.0, 0.0), (0.0, 0.0)], "length": 100.0}]
    assert select_route_aware_lane_projection((20.0, 0.0), 90.0, candidates) is None
    assert (
        select_route_aware_lane_projection((20.0, 10.0), 270.0, candidates, max_distance=8.0)
        is None
    )


def test_route_aware_projection_rejects_overlapping_lane_at_other_elevation():
    from terasim_service.utils.sumo_lane_geometry import (
        select_route_aware_lane_projection,
    )

    candidates = [
        {
            "lane_id": "overpass_0",
            "shape": [(0.0, 0.0), (100.0, 0.0)],
            "shape3d": [(0.0, 0.0, 5.0), (100.0, 0.0, 5.0)],
            "length": 100.0,
        },
        {
            "lane_id": "underpass_0",
            "shape": [(0.0, 0.0), (100.0, 0.0)],
            "shape3d": [(0.0, 0.0, -3.0), (100.0, 0.0, -3.0)],
            "length": 100.0,
        },
    ]

    projection = select_route_aware_lane_projection(
        (25.0, 0.0),
        90.0,
        candidates,
        position_z=-2.9,
        max_elevation_error=2.0,
    )

    assert projection["lane_id"] == "underpass_0"
    assert projection["projected_z"] == pytest.approx(-3.0)
    assert projection["elevation_error"] == pytest.approx(0.1)


def test_curve_adaptive_lookahead_shortens_before_sharp_turn():
    from terasim_service.utils.sumo_lane_geometry import (
        adapt_lookahead_distances_for_compiled_paths,
        compile_lane_shapes,
    )

    straight = compile_lane_shapes([[(0.0, 0.0), (30.0, 0.0)]])
    right_turn = compile_lane_shapes(
        [[(0.0, 0.0), (10.0, 0.0)], [(10.0, 0.0), (10.0, 20.0)]]
    )
    distances, heading_changes = adapt_lookahead_distances_for_compiled_paths(
        [straight, right_turn],
        [(2.0, 0.0), (2.0, 0.0)],
        [15.0, 15.0],
        min_curve_distance=3.5,
    )

    assert distances[0] == pytest.approx(15.0)
    assert heading_changes[0] == pytest.approx(0.0)
    assert distances[1] == pytest.approx(3.5)
    assert heading_changes[1] == pytest.approx(0.5 * 3.141592653589793)


def test_lane_change_lookahead_blends_continuously_with_sumo_heading():
    from terasim_service.utils.sumo_lane_geometry import (
        blend_lane_change_lookahead,
    )

    route_target = (10.0, 3.5, 0.0)
    inactive, inactive_blend = blend_lane_change_lookahead(
        route_target, (0.0, 0.0), 90.0, 10.0
    )
    active, active_blend = blend_lane_change_lookahead(
        route_target,
        (0.0, 0.0),
        84.0,
        10.0,
        lateral_speed=0.35,
        lateral_offset=0.75,
    )

    assert inactive == pytest.approx(route_target)
    assert inactive_blend == pytest.approx(0.0)
    assert active_blend == pytest.approx(1.0)
    assert active[0] == pytest.approx(10.0 * math.cos(math.radians(6.0)))
    assert active[1] == pytest.approx(10.0 * math.sin(math.radians(6.0)))


def test_pure_pursuit_uses_rear_axle_control_point_and_vehicle_wheelbase():
    from terasim_service.utils.carla.ackermann_control import (
        AckermannTuning,
        compute_ackermann_control_values,
    )

    values = compute_ackermann_control_values(
        current_x=10.0,
        current_y=5.0,
        yaw_degrees=0.0,
        current_speed=2.0,
        desired_x=12.0,
        desired_y=5.0,
        lookahead_x=15.0,
        lookahead_y=8.0,
        desired_speed=3.0,
        tuning=AckermannTuning(max_steer_rate_rad_s=0.0),
        control_point_local_x=-1.4,
        wheel_base=2.9,
    )

    assert values.control_point_x == pytest.approx(8.6)
    assert values.control_point_y == pytest.approx(5.0)
    assert values.wheel_base == pytest.approx(2.9)
    assert values.lookahead_local_x == pytest.approx(6.4)
    assert values.lookahead_local_y == pytest.approx(3.0)


def test_pure_pursuit_steers_and_rate_limits():
    from terasim_service.utils.carla.ackermann_control import (
        AckermannTuning,
        compute_ackermann_control_values,
    )

    tuning = AckermannTuning(max_steer_rad=0.6, max_steer_rate_rad_s=0.1)
    values = compute_ackermann_control_values(
        current_x=0.0,
        current_y=0.0,
        yaw_degrees=0.0,
        current_speed=0.0,
        desired_x=10.0,
        desired_y=0.0,
        lookahead_x=10.0,
        lookahead_y=5.0,
        desired_speed=5.0,
        previous_steer=0.0,
        dt=0.1,
        tuning=tuning,
    )
    assert values.steer == pytest.approx(0.01)
    assert values.acceleration == pytest.approx(3.0)


def test_feedback_wildcard_excludes_av_unless_explicit():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_actor_ids = {"*"}
    cosim.ackermann_feedback_all_background_actors = True
    assert cosim._is_ackermann_feedback_selected_actor("BV") is True
    assert cosim._is_ackermann_feedback_selected_actor("AV") is False

    cosim.ackermann_feedback_actor_ids.add("AV")
    assert cosim._is_ackermann_feedback_selected_actor("AV") is True


def test_feedback_apply_enables_physics_only_for_selected_actors():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_physics_enabled = True
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"*"}
    cosim.ackermann_feedback_all_background_actors = True
    assert cosim._uses_ackermann_physics("BV") is True
    assert cosim._uses_ackermann_physics("AV") is False


def test_vehicle_transform_uses_sumo_slope_as_carla_pitch(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla import cosim as cosim_module

    rotations = []

    def convert_transform(location, rotation, shape, offset):
        rotations.append(rotation)
        return FakeTransform(rotation=FakeRotation(pitch=rotation[0]))

    monkeypatch.setattr(cosim_module, "sumo_to_carla", convert_transform)

    transforms = []
    actor = types.SimpleNamespace(id=123, set_transform=transforms.append)
    cosim = cosim_module.CarlaCosim.__new__(cosim_module.CarlaCosim)
    cosim.use_lane_relative_position = False
    cosim.batch_transform_enabled = False
    cosim._resolve_sumo_location = lambda *_args, **_kwargs: [1.0, 2.0, 3.0]
    cosim._resolve_sumo_angle = lambda *_args, **_kwargs: 90.0
    cosim._uses_ackermann_physics = lambda _actor_id: False
    cosim._get_carla_offset = lambda _location, _clearance: [0.0, 0.0, 0.0]
    cosim._ensure_actor_teleport_mode = lambda *_args, **_kwargs: None

    cosim._spawn_failures = {}
    cosim._process_vehicle(
        "BV",
        {
            "x": 1.0,
            "y": 2.0,
            "z": 3.0,
            "sumo_angle": 90.0,
            "sumo_slope": -2.2,
            "length": 5.0,
            "width": 1.8,
            "height": 1.5,
        },
        set(),
        carla_actor=actor,
    )

    assert rotations == [[-2.2, 90.0, 0.0]]
    assert transforms[0].rotation.pitch == pytest.approx(-2.2)


def test_ackermann_controller_settings_are_applied_once(capsys):
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannControllerTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    settings_applied = []
    physics_enabled = []
    actor = types.SimpleNamespace(
        set_simulate_physics=lambda enabled: physics_enabled.append(enabled),
        apply_ackermann_controller_settings=lambda settings: settings_applied.append(settings),
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._ackermann_actor_state = {}
    cosim.ackermann_controller_tuning = AckermannControllerTuning(accel_kp=0.05, accel_kd=0.005)

    cosim._ensure_ackermann_actor_physics(actor, "AV")
    cosim._ensure_ackermann_actor_physics(actor, "AV")

    assert physics_enabled == [True]
    assert len(settings_applied) == 1
    assert settings_applied[0].speed_kp == pytest.approx(1.0)
    assert settings_applied[0].accel_kp == pytest.approx(0.05)
    assert settings_applied[0].accel_kd == pytest.approx(0.005)
    assert cosim._ackermann_actor_state["AV"]["controller_settings_applied"] is True
    assert "CARLA Ackermann controller settings applied" in capsys.readouterr().out


def test_ackermann_spawn_waits_one_carla_tick_then_requires_three_stable_ticks():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannControllerTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    ground_transform = FakeTransform(
        location=FakeLocation(x=100.0, y=200.0, z=4.0),
        rotation=FakeRotation(yaw=90.0),
    )
    elevated_transform = FakeTransform(
        location=FakeLocation(x=100.0, y=200.0, z=9.0),
        rotation=FakeRotation(yaw=90.0),
    )
    frame = [100]
    actor_transform = [elevated_transform]
    actor_velocity = [FakeVector3D()]
    physics_enabled = []
    target_velocities = []
    target_angular_velocities = []

    def set_target_velocity(velocity):
        target_velocities.append(velocity)
        actor_velocity[0] = velocity

    actor = types.SimpleNamespace(
        id=1,
        bounding_box=types.SimpleNamespace(
            location=FakeLocation(),
            rotation=FakeRotation(),
            extent=FakeLocation(x=2.4, y=0.9, z=0.75),
        ),
        get_transform=lambda: actor_transform[0],
        set_transform=lambda transform: actor_transform.__setitem__(0, transform),
        get_velocity=lambda: actor_velocity[0],
        get_physics_control=lambda: types.SimpleNamespace(wheels=[]),
        set_simulate_physics=physics_enabled.append,
        set_target_velocity=set_target_velocity,
        set_target_angular_velocity=target_angular_velocities.append,
        apply_ackermann_controller_settings=lambda _settings: None,
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.world = types.SimpleNamespace(
        get_snapshot=lambda: types.SimpleNamespace(frame=frame[0])
    )
    cosim._ackermann_actor_state = {}
    cosim._vehicle_actor_index = {"AV": actor}
    cosim.spawn_z_clearance = 5.0
    cosim.ackermann_tuning = AckermannTuning()
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    ready = cosim._prepare_ackermann_actor_physics(
        actor,
        "AV",
        8.0,
        ground_transform,
        elevated_transform,
    )
    assert ready is False
    assert actor_transform[0] is ground_transform
    assert physics_enabled == [False]

    # The same CARLA frame must never enable physics after set_transform.
    ready = cosim._ensure_ackermann_actor_physics(
        actor, "AV", initial_speed=8.0, initial_transform=ground_transform
    )
    assert ready is False
    assert physics_enabled == [False]

    # The next frame enables physics, but the actor remains initialization-pending.
    frame[0] = 101
    ready = cosim._ensure_ackermann_actor_physics(
        actor, "AV", initial_speed=8.0, initial_transform=ground_transform
    )
    assert ready is False
    assert physics_enabled == [False, True]
    assert target_velocities[-1].x == pytest.approx(0.0, abs=1e-9)
    assert target_velocities[-1].y == pytest.approx(8.0)

    for expected_frame in (102, 103):
        frame[0] = expected_frame
        assert (
            cosim._ensure_ackermann_actor_physics(
                actor, "AV", initial_speed=8.0, initial_transform=ground_transform
            )
            is False
        )

    frame[0] = 104
    assert (
        cosim._ensure_ackermann_actor_physics(
            actor, "AV", initial_speed=8.0, initial_transform=ground_transform
        )
        is True
    )
    state = cosim._ackermann_actor_state["AV"]
    assert state["physics_initialization_pending"] is False
    assert state["physics_stabilization_pending"] is False
    assert state["physics_stable_ticks"] == 3
    assert len(target_angular_velocities) == 2

def test_ackermann_spawn_defers_physics_while_physical_footprint_overlaps():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannControllerTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    bounding_box = types.SimpleNamespace(
        location=FakeLocation(),
        rotation=FakeRotation(),
        extent=FakeLocation(x=2.4, y=0.9, z=0.75),
    )
    ground_transform = FakeTransform(location=FakeLocation(x=0.0, y=0.0, z=0.0))
    elevated_transform = FakeTransform(location=FakeLocation(x=0.0, y=0.0, z=5.0))
    actor_transform = [elevated_transform]
    blocker_transform = [FakeTransform(location=FakeLocation(x=3.0, y=0.0, z=0.0))]
    frame = [10]
    physics_enabled = []

    actor = types.SimpleNamespace(
        id=1,
        bounding_box=bounding_box,
        get_transform=lambda: actor_transform[0],
        set_transform=lambda transform: actor_transform.__setitem__(0, transform),
        get_velocity=lambda: FakeVector3D(),
        set_simulate_physics=physics_enabled.append,
        set_target_velocity=lambda _velocity: None,
        set_target_angular_velocity=lambda _velocity: None,
        get_physics_control=lambda: types.SimpleNamespace(wheels=[]),
        apply_ackermann_controller_settings=lambda _settings: None,
    )
    blocker = types.SimpleNamespace(
        id=2,
        bounding_box=bounding_box,
        get_transform=lambda: blocker_transform[0],
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.world = types.SimpleNamespace(
        get_snapshot=lambda: types.SimpleNamespace(frame=frame[0])
    )
    cosim._ackermann_actor_state = {"BLOCK": {"physics_enabled": True}}
    cosim._vehicle_actor_index = {"NEW": actor, "BLOCK": blocker}
    cosim.spawn_z_clearance = 5.0
    cosim.ackermann_tuning = AckermannTuning()
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    ready = cosim._prepare_ackermann_actor_physics(
        actor,
        "NEW",
        0.0,
        ground_transform,
        elevated_transform,
    )

    assert ready is False
    assert physics_enabled == [False]
    assert actor_transform[0].location.z == pytest.approx(5.0)
    assert cosim._ackermann_actor_state["NEW"]["physics_overlap_deferred"] is True
    assert cosim._ackermann_actor_state["NEW"]["physics_overlap_blocking_actor"] == "BLOCK"

    blocker_transform[0] = FakeTransform(location=FakeLocation(x=10.0, y=0.0, z=0.0))
    ready = cosim._ensure_ackermann_actor_physics(
        actor,
        "NEW",
        initial_speed=0.0,
        initial_transform=ground_transform,
    )
    assert ready is False
    assert actor_transform[0].location.z == pytest.approx(0.0)

    # It still cannot enable physics until a completed frame follows placement.
    assert (
        cosim._ensure_ackermann_actor_physics(
            actor, "NEW", initial_speed=0.0, initial_transform=ground_transform
        )
        is False
    )
    assert physics_enabled == [False]

    frame[0] = 11
    assert (
        cosim._ensure_ackermann_actor_physics(
            actor, "NEW", initial_speed=0.0, initial_transform=ground_transform
        )
        is False
    )
    assert physics_enabled == [False, True]

    for stable_frame in (12, 13, 14):
        frame[0] = stable_frame
        ready = cosim._ensure_ackermann_actor_physics(
            actor, "NEW", initial_speed=0.0, initial_transform=ground_transform
        )
    assert ready is True
    assert cosim._ackermann_actor_state["NEW"]["physics_initialization_pending"] is False

def test_ackermann_spawn_instability_disables_physics_and_restarts_initialization():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannControllerTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    ground_transform = FakeTransform(location=FakeLocation(x=10.0, y=20.0, z=4.0))
    elevated_transform = FakeTransform(location=FakeLocation(x=10.0, y=20.0, z=9.0))
    actor_transform = [elevated_transform]
    actor_velocity = [FakeVector3D()]
    frame = [20]
    physics_enabled = []
    actor = types.SimpleNamespace(
        id=1,
        bounding_box=types.SimpleNamespace(
            location=FakeLocation(),
            rotation=FakeRotation(),
            extent=FakeLocation(x=2.4, y=0.9, z=0.75),
        ),
        get_transform=lambda: actor_transform[0],
        set_transform=lambda transform: actor_transform.__setitem__(0, transform),
        get_velocity=lambda: actor_velocity[0],
        get_physics_control=lambda: types.SimpleNamespace(wheels=[]),
        set_simulate_physics=physics_enabled.append,
        set_target_velocity=lambda velocity: actor_velocity.__setitem__(0, velocity),
        set_target_angular_velocity=lambda _velocity: None,
        apply_ackermann_controller_settings=lambda _settings: None,
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.world = types.SimpleNamespace(
        get_snapshot=lambda: types.SimpleNamespace(frame=frame[0])
    )
    cosim._ackermann_actor_state = {}
    cosim._vehicle_actor_index = {"BV": actor}
    cosim.spawn_z_clearance = 5.0
    cosim.ackermann_tuning = AckermannTuning()
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    assert (
        cosim._prepare_ackermann_actor_physics(
            actor, "BV", 0.0, ground_transform, elevated_transform
        )
        is False
    )
    frame[0] = 21
    assert (
        cosim._ensure_ackermann_actor_physics(
            actor, "BV", initial_speed=0.0, initial_transform=ground_transform
        )
        is False
    )
    assert physics_enabled == [False, True]

    # Reproduce vehicle2322: the actor is launched upward and gains speed.
    actor_transform[0] = FakeTransform(
        location=FakeLocation(x=10.0, y=20.0, z=5.0),
        rotation=FakeRotation(),
    )
    actor_velocity[0] = FakeVector3D(x=10.0, z=3.0)
    frame[0] = 22
    assert (
        cosim._ensure_ackermann_actor_physics(
            actor, "BV", initial_speed=0.0, initial_transform=ground_transform
        )
        is False
    )

    state = cosim._ackermann_actor_state["BV"]
    assert physics_enabled == [False, True, False]
    assert state["physics_enabled"] is False
    assert state["physics_initialization_pending"] is True
    assert state["physics_overlap_deferred"] is True
    assert state["physics_reinitialization_count"] == 1
    assert actor_transform[0].location.z == pytest.approx(9.0)


def test_ackermann_spawn_abandons_after_three_overlap_failures_without_raising():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannControllerTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    bounding_box = types.SimpleNamespace(
        location=FakeLocation(),
        rotation=FakeRotation(),
        extent=FakeLocation(x=2.4, y=0.9, z=0.75),
    )
    ground_transform = FakeTransform(location=FakeLocation())
    elevated_transform = FakeTransform(location=FakeLocation(z=5.0))
    actor_transform = [elevated_transform]
    physics_enabled = []
    actor = types.SimpleNamespace(
        id=10,
        bounding_box=bounding_box,
        get_transform=lambda: actor_transform[0],
        set_transform=lambda transform: actor_transform.__setitem__(0, transform),
        get_velocity=lambda: FakeVector3D(),
        set_simulate_physics=physics_enabled.append,
        set_target_velocity=lambda _velocity: None,
        set_target_angular_velocity=lambda _velocity: None,
        get_physics_control=lambda: types.SimpleNamespace(wheels=[]),
        apply_ackermann_controller_settings=lambda _settings: None,
    )
    blocker = types.SimpleNamespace(
        id=20,
        bounding_box=bounding_box,
        get_transform=lambda: FakeTransform(location=FakeLocation(x=3.0)),
    )
    destroyed = []
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.world = types.SimpleNamespace(
        get_snapshot=lambda: types.SimpleNamespace(frame=10)
    )
    cosim.client = types.SimpleNamespace(
        apply_batch_sync=lambda commands, _due_tick: destroyed.extend(commands)
    )
    cosim._ackermann_actor_state = {"BLOCK": {"physics_enabled": True}}
    cosim._vehicle_actor_index = {"NEW": actor, "BLOCK": blocker}
    cosim._pending_actor_index_entries = {}
    cosim._spawn_failures = {}
    cosim._collision_sensors = {}
    cosim.spawn_max_attempts = 3
    cosim.spawn_z_clearance = 5.0
    cosim.initialization_diagnostics_enabled = False
    cosim.ackermann_tuning = AckermannTuning()
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    assert (
        cosim._prepare_ackermann_actor_physics(
            actor, "NEW", 0.0, ground_transform, elevated_transform
        )
        is False
    )
    assert (
        cosim._ensure_ackermann_actor_physics(
            actor, "NEW", initial_speed=0.0, initial_transform=ground_transform
        )
        is False
    )
    assert (
        cosim._ensure_ackermann_actor_physics(
            actor, "NEW", initial_speed=0.0, initial_transform=ground_transform
        )
        is False
    )

    assert cosim._spawn_failures[("vehicle", "NEW")]["abandoned"] is True
    assert cosim._ackermann_actor_state["NEW"]["physics_reinitialization_count"] == 3
    assert "NEW" not in cosim._vehicle_actor_index
    assert destroyed == [("destroy", 10)]
    assert cosim._should_retry_spawn(
        "vehicle", "NEW", [0.0, 0.0, 0.0], 11
    ) is False


def test_carla_actor_spawn_failure_is_abandoned_after_three_attempts():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._spawn_failures = {}
    cosim.spawn_max_attempts = 3
    cosim.spawn_failure_backoff_seconds = 0.0
    cosim.spawn_failure_backoff_max_seconds = 0.0

    for frame in range(3):
        cosim._record_spawn_failure("vehicle", "BV", [1.0, 2.0, 0.0], frame)

    failure = cosim._spawn_failures[("vehicle", "BV")]
    assert failure["failures"] == 3
    assert failure["abandoned"] is True
    assert cosim._should_retry_spawn(
        "vehicle", "BV", [1.0, 2.0, 0.0], 100
    ) is False


def test_ackermann_spawn_footprints_allow_separated_adjacent_lane():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    bounding_box = types.SimpleNamespace(
        location=FakeLocation(),
        rotation=FakeRotation(),
        extent=FakeLocation(x=2.4, y=0.9, z=0.75),
    )
    actor = types.SimpleNamespace(bounding_box=bounding_box)
    first = CarlaCosim._ackermann_actor_footprint(
        actor, FakeTransform(location=FakeLocation(x=0.0, y=0.0, z=0.0))
    )
    second = CarlaCosim._ackermann_actor_footprint(
        actor, FakeTransform(location=FakeLocation(x=0.0, y=3.5, z=0.0))
    )

    assert CarlaCosim._ackermann_footprints_overlap(first, second) is False


def test_ackermann_geometry_uses_carla_wheel_positions():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannControllerTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    wheels = [
        types.SimpleNamespace(position=FakeLocation(x=145.0)),
        types.SimpleNamespace(position=FakeLocation(x=145.0)),
        types.SimpleNamespace(position=FakeLocation(x=-135.0)),
        types.SimpleNamespace(position=FakeLocation(x=-135.0)),
    ]
    actor = types.SimpleNamespace(
        bounding_box=types.SimpleNamespace(
            location=FakeLocation(x=0.2),
            extent=FakeLocation(x=2.3),
        ),
        set_simulate_physics=lambda _enabled: None,
        get_physics_control=lambda: types.SimpleNamespace(wheels=wheels),
        apply_ackermann_controller_settings=lambda _settings: None,
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._ackermann_actor_state = {}
    cosim.ackermann_tuning = AckermannTuning(wheel_base=2.8)
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    cosim._ensure_ackermann_actor_physics(actor, "AV")

    state = cosim._ackermann_actor_state["AV"]
    assert state["geometry_from_physics"] is True
    assert state["rear_axle_local_x_m"] == pytest.approx(-1.35)
    assert state["wheel_base_m"] == pytest.approx(2.8)
    assert state["front_bumper_local_x_m"] == pytest.approx(2.5)
    assert state["front_bumper_from_bounding_box"] is True


def test_ackermann_geometry_converts_world_wheel_positions_from_centimeters():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannControllerTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    # At yaw 90 degrees the actor-local x axis is world +y. CARLA 0.9.16 can
    # report these wheel locations as world coordinates in centimetres.
    wheels = [
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=20145.0)),
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=20145.0)),
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=19865.0)),
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=19865.0)),
    ]
    actor = types.SimpleNamespace(
        bounding_box=types.SimpleNamespace(
            location=FakeLocation(x=0.0),
            extent=FakeLocation(x=2.4),
        ),
        get_transform=lambda: FakeTransform(
            location=FakeLocation(x=100.0, y=200.0),
            rotation=FakeRotation(yaw=90.0),
        ),
        set_simulate_physics=lambda _enabled: None,
        get_physics_control=lambda: types.SimpleNamespace(wheels=wheels),
        apply_ackermann_controller_settings=lambda _settings: None,
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._ackermann_actor_state = {}
    cosim.ackermann_tuning = AckermannTuning(wheel_base=2.8)
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    cosim._ensure_ackermann_actor_physics(actor, "AV")

    state = cosim._ackermann_actor_state["AV"]
    assert state["geometry_from_physics"] is True
    assert state["rear_axle_local_x_m"] == pytest.approx(-1.35)
    assert state["wheel_base_m"] == pytest.approx(2.8)


def test_ackermann_geometry_retries_after_spawn_transform_becomes_available():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannControllerTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    wheels = [
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=20145.0)),
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=20145.0)),
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=19865.0)),
        types.SimpleNamespace(position=FakeLocation(x=10000.0, y=19865.0)),
    ]
    transforms = iter(
        [
            FakeTransform(),
            FakeTransform(
                location=FakeLocation(x=100.0, y=200.0),
                rotation=FakeRotation(yaw=90.0),
            ),
        ]
    )
    actor = types.SimpleNamespace(
        bounding_box=types.SimpleNamespace(
            location=FakeLocation(x=0.0),
            extent=FakeLocation(x=2.4),
        ),
        get_transform=lambda: next(transforms),
        set_simulate_physics=lambda _enabled: None,
        get_physics_control=lambda: types.SimpleNamespace(wheels=wheels),
        apply_ackermann_controller_settings=lambda _settings: None,
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._ackermann_actor_state = {}
    cosim.ackermann_tuning = AckermannTuning(wheel_base=2.8)
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    cosim._ensure_ackermann_actor_physics(actor, "AV")
    state = cosim._ackermann_actor_state["AV"]
    assert state.get("geometry_from_physics") is not True

    cosim._initialize_ackermann_actor_geometry(actor, "AV", state)

    assert state["geometry_attempts"] == 2
    assert state["geometry_from_physics"] is True
    assert state["rear_axle_local_x_m"] == pytest.approx(-1.35)
    assert state["wheel_base_m"] == pytest.approx(2.8)


def test_ackermann_reference_transform_round_trips_physical_front_and_rear_axle():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._coord_transformer = None
    cosim.sumo_carla_offset = [100.0, 200.0, 0.0]
    cosim.ackermann_tuning = AckermannTuning(wheel_base=2.8)

    transform = cosim._sumo_front_to_carla_transform(
        [10.0, 20.0, 0.0],
        [0.0, 90.0, 0.0],
        [5.0, 1.8, 1.5],
        [100.0, 200.0, 0.0],
        front_bumper_local_x=2.5,
    )

    assert transform.location.x == pytest.approx(107.5)
    assert transform.location.y == pytest.approx(180.0)
    state = cosim._carla_transform_to_sumo_feedback_state(
        transform,
        [5.0, 1.8, 1.5],
        front_bumper_local_x=2.5,
        rear_axle_local_x=-1.35,
    )
    assert state["position"] == pytest.approx([10.0, 20.0])
    assert state["position_z"] == pytest.approx(0.0)
    assert state["rear_axle_position"] == pytest.approx([6.15, 20.0])
    assert state["sumo_angle"] == pytest.approx(90.0)


def test_ackermann_physics_initializes_velocity_from_sumo_state():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannControllerTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    velocities = []
    transform = FakeTransform(rotation=FakeRotation(yaw=30.0))
    current_velocity = [FakeVector3D()]

    def set_target_velocity(velocity):
        velocities.append(velocity)
        current_velocity[0] = velocity

    actor = types.SimpleNamespace(
        get_transform=lambda: transform,
        get_velocity=lambda: current_velocity[0],
        set_simulate_physics=lambda enabled: None,
        set_target_velocity=set_target_velocity,
        set_target_angular_velocity=lambda _velocity: None,
        apply_ackermann_controller_settings=lambda settings: None,
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._ackermann_actor_state = {}
    cosim.ackermann_controller_tuning = AckermannControllerTuning()

    cosim._ensure_ackermann_actor_physics(actor, "BV", 4.0, transform)
    for _ in range(3):
        cosim._ensure_ackermann_actor_physics(actor, "BV", 4.0, transform)

    nonzero_velocities = [
        velocity for velocity in velocities if math.hypot(velocity.x, velocity.y) > 0.0
    ]
    assert len(nonzero_velocities) == 1
    assert nonzero_velocities[0].x == pytest.approx(4.0 * 3**0.5 / 2.0)
    assert nonzero_velocities[0].y == pytest.approx(2.0)
    assert nonzero_velocities[0].z == pytest.approx(0.0)
    assert cosim._ackermann_actor_state["BV"]["initial_velocity_applied"] is True

def test_actor_radius_filter_uses_exit_hysteresis():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.actor_filter_enabled = True
    cosim.actor_filter_center_id = "AV"
    cosim.actor_filter_radius = 300.0
    cosim.actor_filter_hysteresis = 20.0
    cosim._actor_filter_active_vehicle_ids = set()
    cosim._actor_filter_missing_center_warned = False

    def vehicles_at(distance):
        return {
            "AV": {"x": 0.0, "y": 0.0},
            "BV": {"x": distance, "y": 0.0},
        }

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(299.0), {})
    assert set(filtered) == {"AV", "BV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(310.0), {})
    assert set(filtered) == {"AV", "BV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(321.0), {})
    assert set(filtered) == {"AV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(310.0), {})
    assert set(filtered) == {"AV"}

    filtered, _ = cosim._filter_actor_details_by_radius(vehicles_at(299.0), {})
    assert set(filtered) == {"AV", "BV"}


def test_state_detail_radius_uses_physics_hysteresis():
    from terasim_service.plugins.cosim import TeraSimCoSimPlugin

    plugin = TeraSimCoSimPlugin.__new__(TeraSimCoSimPlugin)
    plugin.state_detail_filter_enabled = True
    plugin.state_detail_radius = 100.0
    plugin.state_detail_hysteresis = 10.0
    plugin.state_filter_center_id = "AV"
    plugin.state_detail_active_vehicle_ids = set()

    def selected_at(distance):
        positions = {
            "AV": (0.0, 0.0, 0.0),
            "BV": (distance, 0.0, 0.0),
        }
        return plugin._update_state_detail_active_vehicle_ids(positions, positions.copy())

    assert selected_at(99.0) == {"AV", "BV"}
    assert selected_at(105.0) == {"AV", "BV"}
    assert selected_at(111.0) == {"AV"}
    assert selected_at(105.0) == {"AV"}
    assert selected_at(99.0) == {"AV", "BV"}


def test_state_detail_filter_disabled_preserves_full_state_contract():
    from terasim_service.plugins.cosim import TeraSimCoSimPlugin

    plugin = TeraSimCoSimPlugin.__new__(TeraSimCoSimPlugin)
    plugin.state_detail_filter_enabled = False
    plugin.state_detail_active_vehicle_ids = {"stale"}

    selected = plugin._update_state_detail_active_vehicle_ids({"AV", "BV"}, {})

    assert selected == {"AV", "BV"}
    assert plugin.state_detail_active_vehicle_ids == {"AV", "BV"}


def _state_subscription_constants():
    return types.SimpleNamespace(
        VAR_DISTANCE=1,
        VAR_POSITION=2,
        VAR_POSITION3D=3,
        VAR_ANGLE=4,
        VAR_SLOPE=15,
        VAR_SPEED=5,
        VAR_SPEED_LAT=14,
        VAR_ACCELERATION=6,
        CMD_GET_VEHICLE_VARIABLE=7,
        VAR_LANE_ID=8,
        VAR_LANEPOSITION=9,
        VAR_LANEPOSITION_LAT=10,
        VAR_NEXT_LINKS=11,
        VAR_ROUTE_ID=12,
        VAR_EDGES=13,
    )


def _state_filter_plugin(plugin_module):
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.state_filter_enabled = True
    plugin.state_filter_radius = 320.0
    plugin.state_filter_center_id = "AV"
    plugin.state_filter_missing_center_logged = False
    plugin.state_filter_error_logged = False
    plugin.state_context_subscription_enabled = True
    plugin.state_context_subscription_active = False
    plugin.state_context_subscription_error_logged = False
    plugin.state_vehicle_context_results = {}
    plugin.logger = types.SimpleNamespace(warning=lambda *args, **kwargs: None)
    return plugin


def test_state_filter_uses_context_subscription_without_per_vehicle_position_queries(
    monkeypatch,
):
    from terasim_service.plugins import cosim as plugin_module

    constants = _state_subscription_constants()
    subscription_calls = []
    results = {
        "AV": {
            constants.VAR_POSITION3D: (0.0, 0.0, 0.0),
            constants.VAR_ANGLE: 90.0,
            constants.VAR_SPEED: 5.0,
            constants.VAR_ACCELERATION: 0.0,
        },
        "near": {
            constants.VAR_POSITION3D: (100.0, 0.0, 0.0),
            constants.VAR_ANGLE: 90.0,
            constants.VAR_SPEED: 4.0,
            constants.VAR_ACCELERATION: -1.0,
        },
    }
    fake_vehicle = types.SimpleNamespace(
        subscribeContext=lambda *args: subscription_calls.append(args),
        getContextSubscriptionResults=lambda _center_id: results,
        getPosition3D=lambda _vehicle_id: (_ for _ in ()).throw(
            AssertionError("per-vehicle position query must not be used")
        ),
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(constants=constants, vehicle=fake_vehicle),
    )
    plugin = _state_filter_plugin(plugin_module)

    selected, positions = plugin._filter_vehicle_ids_for_state(["AV", "near", "far"])

    assert selected == ["AV", "near"]
    assert positions == {
        "AV": (0.0, 0.0, 0.0),
        "near": (100.0, 0.0, 0.0),
    }
    assert subscription_calls == [
        (
            "AV",
            constants.CMD_GET_VEHICLE_VARIABLE,
            320.0,
            [
                constants.VAR_DISTANCE,
                constants.VAR_POSITION,
                constants.VAR_POSITION3D,
                constants.VAR_ANGLE,
                constants.VAR_SLOPE,
                constants.VAR_SPEED,
                constants.VAR_SPEED_LAT,
                constants.VAR_ACCELERATION,
                constants.VAR_LANE_ID,
                constants.VAR_LANEPOSITION,
                constants.VAR_LANEPOSITION_LAT,
                constants.VAR_ROUTE_ID,
                constants.VAR_EDGES,
            ],
        )
    ]


def test_state_filter_falls_back_when_context_subscription_fails(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    constants = _state_subscription_constants()
    positions = {
        "AV": (0.0, 0.0, 0.0),
        "near": (100.0, 0.0, 0.0),
        "far": (400.0, 0.0, 0.0),
    }
    fake_vehicle = types.SimpleNamespace(
        subscribeContext=lambda *args: (_ for _ in ()).throw(RuntimeError("failed")),
        getContextSubscriptionResults=lambda _center_id: None,
        getPosition3D=lambda vehicle_id: positions[vehicle_id],
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(constants=constants, vehicle=fake_vehicle),
    )
    plugin = _state_filter_plugin(plugin_module)

    selected, position_cache = plugin._filter_vehicle_ids_for_state(
        ["AV", "near", "far"]
    )

    assert selected == ["AV", "near"]
    assert position_cache == positions
    assert plugin.state_context_subscription_error_logged is True


def test_detail_profile_helpers_skip_timing_when_profile_is_disabled(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    def unexpected_timer_call():
        raise AssertionError("perf_counter must not run while profiling is disabled")

    monkeypatch.setattr(plugin_module.time, "perf_counter", unexpected_timer_call)
    profile_ctx = {}

    assert plugin_module.TeraSimCoSimPlugin._profile_detail_traci_call(
        profile_ctx, "get_value", lambda value: value + 1, 2
    ) == 3
    assert plugin_module.TeraSimCoSimPlugin._profile_detail_python_call(
        profile_ctx, "geometry", lambda value: value * 2, 3
    ) == 6
    assert "cosim_profile" not in profile_ctx


def test_detail_profile_helpers_record_time_and_traci_count(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    timestamps = iter((1.0, 1.25, 2.0, 2.5))
    monkeypatch.setattr(plugin_module.time, "perf_counter", lambda: next(timestamps))
    profile_ctx = {"cosim_profile": {}}

    plugin_module.TeraSimCoSimPlugin._profile_detail_traci_call(
        profile_ctx, "get_value", lambda: "value"
    )
    plugin_module.TeraSimCoSimPlugin._profile_detail_python_call(
        profile_ctx, "geometry", lambda: "point"
    )

    breakdown = profile_ctx["cosim_profile"]["terasim_internal"]["state_export"][
        "ackermann_detail_breakdown"
    ]
    assert breakdown["traci"] == {
        "total_s": 0.25,
        "get_value_s": 0.25,
        "get_value_calls": 1.0,
    }
    assert breakdown["python"] == {
        "total_s": 0.5,
        "geometry_s": 0.5,
    }


def test_feedback_batches_valid_actors_and_isolates_invalid_shape(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla import cosim as cosim_module
    from terasim_service.utils.carla.cosim import CarlaCosim

    actor = types.SimpleNamespace(
        get_transform=lambda: FakeTransform(FakeLocation(10.0, 20.0, 3.0), FakeRotation()),
        get_velocity=lambda: types.SimpleNamespace(x=3.0, y=4.0, z=12.0),
    )
    batches = []
    monkeypatch.setattr(
        cosim_module,
        "control_agents_batch",
        lambda host, port, simulation_id, commands: (
            batches.append(commands) or {"queued_count": len(commands)}
        ),
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=101))
    cosim.args = types.SimpleNamespace(terasim_host="terasim", terasim_port=8000)
    cosim.terasim = {"simulation_id": "simulation"}
    cosim.terasim_states = {
        "simulation_time": 5.0,
        "agent_details": {
            "vehicle": {
                "good": {"length": 4.0, "width": 2.0, "height": 1.5},
                "bad": {"length": 0.0, "width": 2.0, "height": 1.5},
            }
        },
    }
    cosim.ackermann_feedback_mode = "apply"
    cosim.ackermann_feedback_shadow_enabled = False
    cosim._ackermann_feedback_state = {}
    cosim._ackermann_feedback_candidate_actor_ids = {"good", "bad"}
    cosim._ackermann_feedback_actor_index = {"good": actor, "bad": actor}
    cosim._coord_transformer = None
    cosim.sumo_carla_offset = [0.0, 0.0]

    assert cosim.sync_carla_ackermann_feedback_to_cosim() is False
    assert [[command["agent_id"] for command in batch] for batch in batches] == [["good"]]
    command = batches[0][0]
    assert command["data"]["position"] == pytest.approx([12.0, -20.0])
    assert command["data"]["rear_axle_position"] == pytest.approx([8.6, -20.0])
    assert command["data"]["speed"] == pytest.approx(5.0)
    assert command["data"]["source_carla_frame"] == 101
    assert cosim._ackermann_feedback_state["good"]["feedback_status"] == "queued"
    assert (
        cosim._ackermann_feedback_state["bad"]["feedback_reason"] == "sumo_shape_missing_or_invalid"
    )


def test_http_feedback_tick_orders_feedback_before_next_sumo_step(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla import cosim as cosim_module
    from terasim_service.utils.carla.cosim import CarlaCosim

    events = []
    monkeypatch.setattr(
        cosim_module,
        "tick_terasim",
        lambda *args: events.append("request_sumo_tick"),
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.args = types.SimpleNamespace(skip_tls=True, terasim_host="terasim", terasim_port=8000)
    cosim.terasim = {"simulation_id": "simulation"}
    cosim._wait_for_terasim_step = lambda: "ticked"
    cosim.sync_cosim_actor_to_carla = lambda: events.append("apply_sumo_state")
    cosim.world = types.SimpleNamespace(tick=lambda: events.append("carla_tick"))
    cosim.sync_carla_ackermann_feedback_to_cosim = lambda: events.append("queue_feedback")

    assert cosim._tick_ackermann_feedback_apply_http() is True
    assert events == [
        "apply_sumo_state",
        "carla_tick",
        "queue_feedback",
        "request_sumo_tick",
    ]


def test_grpc_feedback_is_attached_to_tick_after_carla_frame(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    direct_link_module = types.SimpleNamespace(
        parse_state_json=lambda value: json.loads(value) if value else None
    )
    monkeypatch.setitem(
        sys.modules,
        "terasim_service.utils.carla.direct_link",
        direct_link_module,
    )
    monkeypatch.setitem(sys.modules, "grpc", types.SimpleNamespace(RpcError=Exception))

    events = []
    command = {
        "agent_id": "BV",
        "agent_type": "vehicle",
        "command_type": "set_state",
        "data": {"source_carla_frame": 7},
    }
    record = {
        "actor_id": "BV",
        "source_carla_frame": 7,
        "feedback_status": "rejected",
    }
    future = object()
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._direct_tick_future = None
    cosim._direct_prev_state = {"agent_details": {"vehicle": {}, "vru": {}}}
    cosim.args = types.SimpleNamespace(skip_tls=True)
    cosim.sync_cosim_actor_to_carla = lambda state: events.append("apply_sumo_state")
    cosim.world = types.SimpleNamespace(tick=lambda: events.append("carla_tick"))
    cosim._collect_ackermann_feedback = lambda: (
        events.append("collect_feedback") or ([command], [record])
    )
    cosim.direct_link = types.SimpleNamespace(
        tick_async=lambda commands: (events.append(("grpc_tick", commands)) or future)
    )
    cosim._ackermann_feedback_state = {}
    cosim._record_ackermann_feedback = lambda feedback: events.append(("record", feedback.copy()))

    assert cosim._tick_ackermann_feedback_apply_direct() is True
    assert cosim._direct_tick_future is future
    assert events[0:3] == [
        "apply_sumo_state",
        "carla_tick",
        "collect_feedback",
    ]
    assert events[3] == ("grpc_tick", [command])
    assert events[4][1]["feedback_status"] == "queued"
    assert events[4][1]["feedback_reason"] == "accepted_by_grpc_tick"


def test_feedback_wait_requires_new_completed_tick(monkeypatch):
    install_fake_carla()
    from terasim_service.utils.carla import cosim as cosim_module
    from terasim_service.utils.carla.cosim import CarlaCosim

    responses = iter(
        [
            {"status": "ticked", "completed_tick_count": 7},
            {"status": "running", "completed_tick_count": 7},
            {"status": "ticked", "completed_tick_count": 8},
        ]
    )
    monkeypatch.setattr(cosim_module, "get_terasim_status", lambda *args: next(responses))
    monkeypatch.setattr(cosim_module.time, "sleep", lambda seconds: None)

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.args = types.SimpleNamespace(terasim_host="terasim", terasim_port=8000)
    cosim.terasim = {"simulation_id": "simulation"}
    cosim._initial_terasim_state_pending = False
    cosim._last_completed_terasim_tick_count = 7

    assert cosim._wait_for_terasim_step() == "ticked"
    assert cosim._last_completed_terasim_tick_count == 8


def test_feedback_lc_keep_right_is_applied_once_and_verified(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    calls = []
    parameters = {}

    def set_parameter(actor_id, name, value):
        calls.append((actor_id, name, value))
        parameters[(actor_id, name)] = value

    fake_vehicle = types.SimpleNamespace(
        setParameter=set_parameter,
        getParameter=lambda actor_id, name: parameters[(actor_id, name)],
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.ackermann_feedback_lc_keep_right = 0.0
    plugin.ackermann_feedback_lc_keep_right_actor_ids = {"AV"}
    plugin.ackermann_feedback_lane_change_settings_applied = set()
    plugin.logger = types.SimpleNamespace(info=lambda *args, **kwargs: None)

    plugin._ensure_ackermann_feedback_lane_change_settings("AV")
    plugin._ensure_ackermann_feedback_lane_change_settings("AV")
    plugin._ensure_ackermann_feedback_lane_change_settings("BV")

    assert calls == [("AV", "laneChangeModel.lcKeepRight", "0")]
    assert plugin.ackermann_feedback_lane_change_settings_applied == {"AV"}


def test_feedback_ack_is_cached_by_shared_sumo_command_handler(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    calls = []
    command = types.SimpleNamespace(
        agent_id="AV",
        agent_type="vehicle",
        command_type="set_state",
        data={
            "position": [1.0, 2.0],
            "rear_axle_position": [0.25, 2.0],
            "sumo_angle": 90.0,
            "speed": 3.5,
            "source_carla_frame": 101,
        },
    )
    lane_state = {
        "road_id": "edge_0",
        "lane_id": "edge_0_0",
        "lane_position": 10.0,
        "route_index": 0,
    }
    warnings = []

    def move_to(*args):
        calls.append(("move", args))
        if args[2] == -1:
            lane_state.update(
                road_id="edge_0",
                lane_id="edge_0_1",
                lane_position=11.0,
                route_index=0,
            )

    fake_vehicle = types.SimpleNamespace(
        moveTo=move_to,
        setPreviousSpeed=lambda *args: calls.append(("speed", args)),
        getIDList=lambda: [],
        getRoadID=lambda _actor_id: lane_state["road_id"],
        getLaneID=lambda _actor_id: lane_state["lane_id"],
        getLanePosition=lambda _actor_id: lane_state["lane_position"],
        getRouteIndex=lambda _actor_id: lane_state["route_index"],
    )
    monkeypatch.setattr(plugin_module, "traci", types.SimpleNamespace(vehicle=fake_vehicle))
    monkeypatch.setattr(
        plugin_module.AgentCommand,
        "model_validate_json",
        staticmethod(lambda payload: command),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.controlled_agents_each_step = set()
    plugin.feedback_observed_speeds = {}
    plugin.feedback_observed_rear_axle_positions = {}
    plugin.feedback_source_carla_frames = {}
    plugin.ackermann_feedback_position_mode = "moveToXY"
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        debug=lambda *args, **kwargs: None,
        warning=lambda message, *args, **kwargs: warnings.append(message % args),
        error=lambda *args, **kwargs: None,
    )

    plugin._move_ackermann_feedback_actor = (
        lambda actor_id, position, _sumo_angle, **_kwargs: (
            calls.append(("move", (actor_id, "edge_0_0", position[0])))
            or {"lane_id": "edge_0_0", "lane_position": position[0]}
        )
    )

    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0] == ("move", ("AV", "edge_0_0", 1.0))
    assert plugin.feedback_observed_speeds == {"AV": 3.5}
    assert plugin.feedback_observed_rear_axle_positions == {"AV": (0.25, 2.0)}
    assert plugin.feedback_source_carla_frames == {"AV": 101}
    assert calls[-1] == ("speed", ("AV", 3.5))


def _external_state_handler_plugin(plugin_module):
    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.controlled_agents_each_step = set()
    plugin.feedback_observed_speeds = {}
    plugin.feedback_observed_rear_axle_positions = {}
    plugin.feedback_source_carla_frames = {}
    plugin.feedback_observed_lane_progress = {}
    plugin.feedback_lane_geometry_cache = {"road_0": {"length": 500.0}}
    plugin.feedback_lane_change_active_actor_ids = set()
    plugin.ackermann_feedback_lane_change_settings_applied = set()
    plugin.ackermann_feedback_lc_keep_right = None
    plugin.ackermann_feedback_assimilation_mode = "external_state"
    plugin.ackermann_feedback_validate_external_state = True
    plugin.ackermann_feedback_external_state_position_tolerance = 1e-3
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_log_lane_transitions = False
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        debug=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )
    return plugin


def test_external_state_feedback_is_immediate_and_does_not_step(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    events = []
    state = {
        "time": 12.5,
        "position": (0.0, 0.0),
        "angle": 0.0,
        "speed": 0.0,
    }
    command = types.SimpleNamespace(
        agent_id="AV",
        agent_type="vehicle",
        command_type="set_state",
        data={
            "position": [41.0, 2.0],
            "sumo_angle": 90.0,
            "speed": 8.5,
            "acceleration": -1.25,
            "source_carla_frame": 101,
        },
    )

    def set_external_state(*args):
        events.append(("external_state", args))
        state["position"] = (args[3], args[4])
        state["angle"] = args[5]
        state["speed"] = args[6]

    def unexpected_previous_speed(*_args):
        raise AssertionError("setPreviousSpeed must not follow setExternalState")

    fake_vehicle = types.SimpleNamespace(
        setExternalState=set_external_state,
        setPreviousSpeed=unexpected_previous_speed,
        getPosition=lambda _actor_id: state["position"],
        getAngle=lambda _actor_id: state["angle"],
        getSpeed=lambda _actor_id: state["speed"],
    )
    fake_simulation = types.SimpleNamespace(
        getTime=lambda: events.append(("time", state["time"])) or state["time"]
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, simulation=fake_simulation),
    )
    monkeypatch.setattr(
        plugin_module.AgentCommand,
        "model_validate_json",
        staticmethod(lambda payload: command),
    )

    plugin = _external_state_handler_plugin(plugin_module)

    def project(actor_id, position, sumo_angle, **kwargs):
        events.append(("project", actor_id, position, sumo_angle, kwargs))
        return {
            "lane_id": "road_0",
            "edge_id": "road",
            "lane_index": 0,
            "lane_position": 41.0,
        }

    plugin._move_ackermann_feedback_actor = project

    assert plugin._handle_agent_command(b"{}") is True
    assert events[0][0] == "project"
    assert events[0][4]["apply_position"] is False
    external_calls = [event for event in events if event[0] == "external_state"]
    assert external_calls == [
        (
            "external_state",
            ("AV", "road", 0, 41.0, 2.0, 90.0, 8.5, -1.25, 1, 8.0),
        )
    ]
    assert [event for event in events if event[0] == "time"] == [
        ("time", 12.5),
        ("time", 12.5),
    ]
    assert state == {
        "time": 12.5,
        "position": (41.0, 2.0),
        "angle": 90.0,
        "speed": 8.5,
    }
    assert plugin.feedback_observed_speeds == {"AV": 8.5}
    assert plugin.feedback_source_carla_frames == {"AV": 101}


def test_external_state_validation_accepts_geometry_rounding(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    plugin = _external_state_handler_plugin(plugin_module)
    fake_vehicle = types.SimpleNamespace(
        setExternalState=lambda *_args: None,
        getPosition=lambda _actor_id: (41.0005, 2.0),
        getAngle=lambda _actor_id: 90.0,
        getSpeed=lambda _actor_id: 8.5,
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(
            vehicle=fake_vehicle,
            simulation=types.SimpleNamespace(getTime=lambda: 12.5),
        ),
    )

    assert plugin._apply_ackermann_feedback_external_state(
        "AV",
        (41.0, 2.0),
        90.0,
        8.5,
        -1.25,
        {"edge_id": "road", "lane_index": 0},
    )


def test_external_state_mode_fails_closed_without_dedicated_api(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    command = types.SimpleNamespace(
        agent_id="AV",
        agent_type="vehicle",
        command_type="set_state",
        data={
            "position": [41.0, 2.0],
            "sumo_angle": 90.0,
            "speed": 8.5,
            "source_carla_frame": 101,
        },
    )
    fake_vehicle = types.SimpleNamespace(
        setPreviousSpeed=lambda *_args: (_ for _ in ()).throw(
            AssertionError("speed must not be applied after API capability failure")
        )
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle),
    )
    monkeypatch.setattr(
        plugin_module.AgentCommand,
        "model_validate_json",
        staticmethod(lambda payload: command),
    )

    plugin = _external_state_handler_plugin(plugin_module)
    plugin._move_ackermann_feedback_actor = lambda *_args, **_kwargs: {
        "lane_id": "road_0",
        "edge_id": "road",
        "lane_index": 0,
        "lane_position": 41.0,
    }

    assert plugin._handle_agent_command(b"{}") is False
    assert plugin.last_agent_command_failure == {
        "actor_id": "AV",
        "reason": "ackermann_feedback_external_state_api_unavailable",
        "ackermann_feedback": True,
    }


def test_feedback_move_to_is_immediate_and_preserves_current_sumo_lane(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    calls = []
    command = types.SimpleNamespace(
        agent_id="BV",
        agent_type="vehicle",
        command_type="set_state",
        data={
            "position": [25.0, 2.8],
            "sumo_angle": 90.0,
            "speed": 3.5,
            "source_carla_frame": 101,
        },
    )
    lane_state = {
        "road_id": "edge_0",
        "lane_id": "edge_0_0",
        "lane_position": 10.0,
        "route_index": 0,
    }
    lane_shapes = {
        "edge_0_0": [(0.0, 0.0), (100.0, 0.0)],
        "edge_0_1": [(0.0, 3.2), (100.0, 3.2)],
        ":junction_0_0": [(100.0, 0.0), (105.0, 0.0)],
        "edge_1_0": [(105.0, 0.0), (205.0, 0.0)],
    }

    def move_to(actor_id, lane_id, lane_position):
        calls.append(("move", (actor_id, lane_id, lane_position)))
        lane_state.update(
            road_id=lane_id.rsplit("_", 1)[0],
            lane_id=lane_id,
            lane_position=lane_position,
        )

    fake_vehicle = types.SimpleNamespace(
        moveTo=move_to,
        moveToXY=lambda *args: calls.append(("move_xy", args)),
        setPreviousSpeed=lambda *args: calls.append(("speed", args)),
        getIDList=lambda: ["BV"],
        getRoadID=lambda _actor_id: lane_state["road_id"],
        getLaneID=lambda _actor_id: lane_state["lane_id"],
        getLanePosition=lambda _actor_id: lane_state["lane_position"],
        getRouteIndex=lambda _actor_id: lane_state["route_index"],
        getRoute=lambda _actor_id: ("edge_0", "edge_1"),
        setRoute=lambda actor_id, route: calls.append(
            ("set_route", (actor_id, tuple(route)))
        ),
        getNextLinks=lambda _actor_id: [
            ("edge_1_0", ":junction_0_0", True, True, False, "G", "s", 5.0)
        ],
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: lane_shapes[lane_id],
        getLength=lambda lane_id: 5.0 if lane_id.startswith(":") else 100.0,
    )
    fake_edge = types.SimpleNamespace(
        getLaneNumber=lambda edge_id: 2 if edge_id == "edge_0" else 1,
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane, edge=fake_edge),
    )
    monkeypatch.setattr(
        plugin_module.AgentCommand,
        "model_validate_json",
        staticmethod(lambda payload: command),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.controlled_agents_each_step = set()
    plugin.feedback_observed_speeds = {}
    plugin.feedback_source_carla_frames = {}
    plugin.feedback_lane_states = {}
    plugin.ackermann_feedback_position_mode = "moveToXY"
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_log_lane_transitions = True
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        debug=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0][0] == "move"
    assert calls[0][1][0:2] == ("BV", "edge_0_0")
    assert calls[0][1][2] == pytest.approx(25.0)
    assert calls[1] == ("speed", ("BV", 3.5))
    assert not any(call[0] == "move_xy" for call in calls)
    assert plugin.feedback_lane_states["BV"]["lane_id"] == "edge_0_0"

    calls.clear()
    plugin.controlled_agents_each_step.clear()
    plugin.feedback_lane_change_active_actor_ids = {"BV"}
    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0][0] == "move"
    assert calls[0][1][0:2] == ("BV", "edge_0_1")
    assert calls[0][1][2] == pytest.approx(25.0)
    assert calls[1] == ("speed", ("BV", 3.5))
    assert not any(call[0] == "move_xy" for call in calls)
    plugin.feedback_lane_change_active_actor_ids.clear()
    lane_state.update(
        road_id="edge_0",
        lane_id="edge_0_0",
        lane_position=25.0,
    )

    # Even when CARLA is closer to the adjacent lane, moveTo must not switch
    # lanes on CARLA position. An invalid current-lane projection is rejected.
    calls.clear()
    plugin.controlled_agents_each_step.clear()
    command.data["position"] = [25.0, 10.0]
    assert plugin._handle_agent_command(b"{}") is False
    assert calls == []
    assert plugin.last_agent_command_failure["reason"] == (
        "ackermann_feedback_moveTo_mapping_failed"
    )

    # A wider background-only tolerance does not relax the AV limit.
    plugin.ackermann_feedback_background_move_to_max_distance = 20.0
    plugin.last_agent_command_failure = None
    plugin.controlled_agents_each_step.clear()
    assert plugin._handle_agent_command(b"{}") is True
    assert calls[0][0] == "move"
    assert calls[0][1][0:2] == ("BV", "edge_0_0")
    assert calls[0][1][2] == pytest.approx(25.0)


def test_feedback_lane_change_is_limited_to_current_edge_and_elevation(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    lane_shapes = {
        "edge_1603_0": [(0.0, 0.0), (100.0, 0.0)],
        "edge_1603_1": [(0.0, 3.2), (100.0, 3.2)],
        ":crossing_0_0": [(25.0, -20.0), (25.0, 20.0)],
    }
    lane_shapes_3d = {
        "edge_1603_0": [(0.0, 0.0, -3.0), (100.0, 0.0, -3.0)],
        "edge_1603_1": [(0.0, 3.2, -3.0), (100.0, 3.2, -3.0)],
        ":crossing_0_0": [(25.0, -20.0, 5.0), (25.0, 20.0, 5.0)],
    }
    lane_edges = {
        "edge_1603_0": "edge_1603",
        "edge_1603_1": "edge_1603",
        ":crossing_0_0": ":crossing_0",
    }
    lane_state = {"lane_id": "edge_1603_1"}
    moves = []
    routes = []

    def move_to(*args):
        moves.append(args)
        lane_state["lane_id"] = args[1]

    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _actor_id: lane_state["lane_id"],
        getNextLinks=lambda _actor_id: [],
        getRoute=lambda _actor_id: ("edge_1603", "edge_next"),
        getRouteIndex=lambda _actor_id: 0,
        setRoute=lambda actor_id, route: routes.append((actor_id, tuple(route))),
        moveTo=move_to,
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: lane_shapes[lane_id],
        getLength=lambda _lane_id: 100.0,
        getEdgeID=lambda lane_id: lane_edges[lane_id],
    )
    fake_edge = types.SimpleNamespace(
        getLaneNumber=lambda edge_id: 2 if edge_id == "edge_1603" else 1,
    )
    fake_net = types.SimpleNamespace(
        getLane=lambda lane_id: types.SimpleNamespace(
            getShape3D=lambda: lane_shapes_3d[lane_id]
        )
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane, edge=fake_edge),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.simulator = types.SimpleNamespace(sumo_net=fake_net)
    plugin.feedback_lane_geometry_cache = {}
    plugin.feedback_edge_lane_ids_cache = {}
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_max_elevation_error = 2.0
    plugin.logger = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    projection = plugin._move_ackermann_feedback_actor_exact(
        "BV",
        (25.0, 3.1),
        90.0,
        position_z=-2.9,
    )

    assert projection["lane_id"] == "edge_1603_1"
    assert routes == []
    assert len(moves) == 1
    assert moves[0][0:2] == ("BV", "edge_1603_1")
    assert moves[0][2] == pytest.approx(25.0)
    assert lane_state["lane_id"] == "edge_1603_1"


def test_feedback_lane_change_uses_selected_move_to_lane(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    lane_state = {"lane_id": "edge_0_0"}
    moves = []

    def move_to(*args):
        moves.append(args)
        lane_state["lane_id"] = args[1]

    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _actor_id: lane_state["lane_id"],
        getNextLinks=lambda _actor_id: [],
        getRoute=lambda _actor_id: ("edge_0", "edge_1"),
        getRouteIndex=lambda _actor_id: 0,
        setRoute=lambda _actor_id, _route: None,
        moveTo=move_to,
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda _lane_id: [(0.0, 0.0), (100.0, 0.0)],
        getLength=lambda _lane_id: 100.0,
        getEdgeID=lambda lane_id: lane_id.rsplit("_", 1)[0],
    )
    fake_edge = types.SimpleNamespace(getLaneNumber=lambda _edge_id: 1)
    fake_net = types.SimpleNamespace(
        getLane=lambda lane_id: types.SimpleNamespace(
            getShape3D=lambda: (
                [(0.0, 0.0, 5.0), (100.0, 0.0, 5.0)]
                if lane_id.startswith(":overpass")
                else [(0.0, 0.0, -3.0), (100.0, 0.0, -3.0)]
            )
        )
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane, edge=fake_edge),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.simulator = types.SimpleNamespace(sumo_net=fake_net)
    plugin.feedback_lane_geometry_cache = {}
    plugin.feedback_edge_lane_ids_cache = {}
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_max_elevation_error = 2.0
    plugin.last_agent_command_failure = None
    plugin.logger = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    projection = plugin._move_ackermann_feedback_actor_exact(
        "BV",
        (25.0, 0.0),
        90.0,
        position_z=-3.0,
    )

    assert projection is not None
    assert projection["lane_id"] == "edge_0_0"
    assert len(moves) == 1
    assert moves[0][0:2] == ("BV", "edge_0_0")
    assert moves[0][2] == pytest.approx(25.0)
    assert lane_state["lane_id"] == "edge_0_0"
    assert plugin.last_agent_command_failure is None


def test_feedback_lane_change_preserves_signal_stop_line_clamp(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    lane_shapes = {
        "edge_0_0": [(0.0, 0.0), (10.0, 0.0)],
        ":junction_0_0": [(10.0, 0.0), (15.0, 0.0)],
        "edge_1_0": [(15.0, 0.0), (25.0, 0.0)],
    }
    lane_edges = {
        "edge_0_0": "edge_0",
        ":junction_0_0": ":junction_0",
        "edge_1_0": "edge_1",
    }
    lane_state = {"lane_id": "edge_0_0"}
    moves = []
    routes = []

    def move_to(*args):
        moves.append(args)
        lane_state["lane_id"] = args[1]

    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _actor_id: lane_state["lane_id"],
        getNextLinks=lambda _actor_id: [
            ("edge_1_0", ":junction_0_0", True, True, False, "r", "s", 5.0)
        ],
        getRoute=lambda _actor_id: ("edge_0", "edge_1"),
        getRouteIndex=lambda _actor_id: 0,
        setRoute=lambda actor_id, route: routes.append((actor_id, tuple(route))),
        moveTo=move_to,
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: lane_shapes[lane_id],
        getLength=lambda lane_id: 5.0 if lane_id.startswith(":") else 10.0,
        getEdgeID=lambda lane_id: lane_edges[lane_id],
    )
    fake_edge = types.SimpleNamespace(getLaneNumber=lambda _edge_id: 1)
    fake_net = types.SimpleNamespace(
        getLane=lambda lane_id: types.SimpleNamespace(
            getShape3D=lambda: [
                (point[0], point[1], 0.0) for point in lane_shapes[lane_id]
            ]
        )
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane, edge=fake_edge),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.simulator = types.SimpleNamespace(sumo_net=fake_net)
    plugin.feedback_lane_geometry_cache = {}
    plugin.feedback_edge_lane_ids_cache = {}
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_max_elevation_error = 2.0
    plugin.ackermann_feedback_signal_stop_line_clamp_offset = 1.01
    plugin.logger = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    plugin._move_ackermann_feedback_actor_exact(
        "BV", (11.0, 0.0), 90.0, position_z=0.0
    )
    plugin._move_ackermann_feedback_actor_exact(
        "BV", (11.01, 0.0), 90.0, position_z=0.0
    )
    plugin._move_ackermann_feedback_actor_exact(
        "BV", (11.02, 0.0), 90.0, position_z=0.0
    )

    assert moves[0][0:2] == ("BV", "edge_0_0")
    assert moves[0][2] == pytest.approx(10.0)
    assert moves[1][0:2] == ("BV", "edge_0_0")
    assert moves[1][2] == pytest.approx(10.0)
    assert moves[2][0:2] == ("BV", ":junction_0_0")
    assert moves[2][2] == pytest.approx(1.02)
    assert routes == []
    assert lane_state["lane_id"] == ":junction_0_0"


def test_feedback_move_to_signal_stop_line_clamps_1_01_meter_then_uses_successor(
    monkeypatch,
):
    from terasim_service.plugins import cosim as plugin_module

    lane_shapes = {
        "edge_0_0": [(0.0, 0.0), (10.0, 0.0)],
        ":junction_0_0": [(10.0, 0.0), (15.0, 0.0)],
        "edge_1_0": [(15.0, 0.0), (25.0, 0.0)],
    }
    lane_lengths = {
        "edge_0_0": 10.0,
        ":junction_0_0": 2.5,
        "edge_1_0": 10.0,
    }
    moves = []
    link_state = {"value": "r"}
    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _actor_id: "edge_0_0",
        getNextLinks=lambda _actor_id: [
            (
                "edge_1_0",
                ":junction_0_0",
                True,
                True,
                False,
                link_state["value"],
                "s",
                5.0,
            )
        ],
        moveTo=lambda *args: moves.append(args),
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: lane_shapes[lane_id],
        getLength=lambda lane_id: lane_lengths[lane_id],
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_lane_geometry_cache = {}
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_signal_stop_line_clamp_offset = 1.01
    plugin.logger = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    plugin._move_ackermann_feedback_actor("BV", (11.0, 0.0), 90.0)
    plugin._move_ackermann_feedback_actor("BV", (11.01, 0.0), 90.0)
    plugin._move_ackermann_feedback_actor("BV", (11.02, 0.0), 90.0)

    assert moves[0] == ("BV", "edge_0_0", pytest.approx(10.0))
    assert moves[1] == ("BV", "edge_0_0", pytest.approx(10.0))
    assert moves[2] == ("BV", ":junction_0_0", pytest.approx(0.51))

    link_state["value"] = "M"
    plugin._move_ackermann_feedback_actor("BV", (10.05, 0.0), 90.0)
    assert moves[3] == ("BV", ":junction_0_0", pytest.approx(0.025))

    link_state["value"] = "G"
    plugin._move_ackermann_feedback_actor("BV", (10.05, 0.0), 90.0)
    assert moves[4] == ("BV", ":junction_0_0", pytest.approx(0.025))


def test_feedback_move_to_does_not_apply_signal_clamp_after_internal_lane(
    monkeypatch,
):
    from terasim_service.plugins import cosim as plugin_module

    lane_shapes = {
        ":junction_0_0": [(10.0, 0.0), (15.0, 0.0)],
        "edge_1_0": [(15.0, 0.0), (25.0, 0.0)],
    }
    moves = []
    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _actor_id: ":junction_0_0",
        getNextLinks=lambda _actor_id: [
            ("edge_1_0", "", True, True, False, "r", "s", 5.0)
        ],
        moveTo=lambda *args: moves.append(args),
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: lane_shapes[lane_id],
        getLength=lambda lane_id: 5.0 if lane_id.startswith(":") else 10.0,
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_lane_geometry_cache = {}
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.ackermann_feedback_signal_stop_line_clamp_offset = 1.01
    plugin.logger = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    plugin._move_ackermann_feedback_actor("BV", (15.5, 0.0), 90.0)

    assert moves == [("BV", "edge_1_0", pytest.approx(0.5))]


def test_feedback_move_to_accepts_current_internal_lane_before_body_finishes_turn(
    monkeypatch,
):
    from terasim_service.plugins import cosim as plugin_module

    moves = []
    lane_id = ":node_53_0_0"
    lane_shape = [
        (89266.418, 43248.115),
        (89266.147, 43247.677),
        (89264.813, 43247.398),
        (89264.564, 43247.896),
    ]
    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda _actor_id: lane_id,
        moveTo=lambda *args: moves.append(args),
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda _lane_id: lane_shape,
        getLength=lambda _lane_id: 2.478,
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(vehicle=fake_vehicle, lane=fake_lane),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_lane_geometry_cache = {}
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.logger = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    projection = plugin._move_ackermann_feedback_actor(
        "vehicle2441",
        (89266.375, 43248.161),
        334.472,
    )

    assert projection is not None
    assert projection["lane_id"] == lane_id
    assert projection["distance"] < 0.1
    assert projection["heading_error"] > 90.0
    assert moves == [("vehicle2441", lane_id, pytest.approx(projection["lane_position"]))]


def test_feedback_move_to_uses_only_current_lane_and_profiles_calls(monkeypatch):
    from terasim_service.plugins import cosim as plugin_module

    calls = []

    def unexpected(*_args):
        raise AssertionError("route and adjacent-lane TraCI calls are unnecessary")

    fake_vehicle = types.SimpleNamespace(
        getLaneID=lambda actor_id: calls.append(("get_lane", actor_id)) or "edge_0_0",
        moveTo=lambda *args: calls.append(("move", args)),
        getRoadID=unexpected,
        getRoute=unexpected,
        getRouteIndex=unexpected,
        getNextLinks=unexpected,
    )
    fake_lane = types.SimpleNamespace(
        getShape=lambda lane_id: calls.append(("shape", lane_id))
        or [(0.0, 0.0), (100.0, 0.0)],
        getLength=lambda lane_id: calls.append(("length", lane_id)) or 100.0,
    )
    monkeypatch.setattr(
        plugin_module,
        "traci",
        types.SimpleNamespace(
            vehicle=fake_vehicle,
            lane=fake_lane,
            edge=types.SimpleNamespace(getLaneNumber=unexpected),
        ),
    )

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.feedback_lane_geometry_cache = {}
    plugin.ackermann_feedback_move_to_max_distance = 8.0
    plugin.ackermann_feedback_background_move_to_max_distance = None
    plugin.ackermann_feedback_move_to_lane_hysteresis = 0.35
    plugin.logger = types.SimpleNamespace(
        debug=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )
    profile_ctx = {"cosim_profile": {"terasim_internal": {}}}

    projection = plugin._move_ackermann_feedback_actor(
        "BV", (25.0, 0.0), 90.0, profile_ctx
    )
    assert projection["lane_id"] == "edge_0_0"
    assert [name for name, _value in calls] == ["get_lane", "shape", "length", "move"]

    calls.clear()
    plugin._move_ackermann_feedback_actor("BV", (26.0, 0.0), 90.0, profile_ctx)
    assert [name for name, _value in calls] == ["get_lane", "move"]
    breakdown = profile_ctx["cosim_profile"]["terasim_internal"][
        "feedback_command_breakdown"
    ]
    assert breakdown["lane_geometry_cache_misses"] == pytest.approx(1.0)
    assert breakdown["lane_geometry_cache_hits"] == pytest.approx(1.0)
    assert breakdown["traci"]["vehicle_get_lane_id_calls"] == pytest.approx(2.0)
    assert breakdown["traci"]["vehicle_move_to_calls"] == pytest.approx(2.0)


def test_ackermann_feedback_uses_same_tick_position_speed_and_acceleration():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"AV"}
    cosim.ackermann_feedback_all_background_actors = False
    cosim.step_length = 0.05
    cosim.ackermann_tuning = AckermannTuning(
        position_speed_gain=1.0,
        kp_speed=0.8,
        kp_position=0.15,
        max_accel=3.0,
        max_decel=6.0,
    )
    cosim._ackermann_actor_state = {}

    target, acceleration = cosim._resolve_ackermann_longitudinal_target(
        "AV",
        {
            "speed": 1.2,
            "sumo_desired_speed": 1.2,
            "feedback_observed_speed": 1.0,
            "acceleration": 1.5,
        },
        current_speed=1.0,
        longitudinal_error=0.5,
    )
    assert acceleration == pytest.approx(1.735)
    assert target == pytest.approx(1.7)
    assert cosim._ackermann_actor_state["AV"]["restart_active"] is False
    assert "restart_target_speed" not in cosim._ackermann_actor_state["AV"]


def _make_ackermann_restart_test_cosim(actor_id):
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = True
    if actor_id == "AV":
        cosim.ackermann_feedback_actor_ids = {"AV"}
        cosim.ackermann_feedback_all_background_actors = False
    else:
        cosim.ackermann_feedback_actor_ids = {"*"}
        cosim.ackermann_feedback_all_background_actors = True
    cosim.step_length = 0.05
    cosim.ackermann_tuning = AckermannTuning()
    cosim._ackermann_actor_state = {}
    return cosim


@pytest.mark.parametrize("actor_id", ["AV", "vehicle123"])
def test_ackermann_restart_target_accumulates_for_av_and_background(actor_id):
    cosim = _make_ackermann_restart_test_cosim(actor_id)
    veh_info = {
        "speed": 0.092,
        "sumo_desired_speed": 0.092,
        "feedback_observed_speed": 0.0,
        "acceleration": 1.84,
    }

    targets = [
        cosim._resolve_ackermann_longitudinal_target(
            actor_id,
            veh_info,
            current_speed=0.0,
        )[0]
        for _ in range(4)
    ]

    assert targets == pytest.approx([0.092, 0.184, 0.276, 0.3])
    assert cosim._ackermann_actor_state[actor_id]["restart_active"] is True
    assert cosim._ackermann_actor_state[actor_id]["restart_target_speed"] == pytest.approx(
        0.3
    )


@pytest.mark.parametrize("actor_id", ["AV", "vehicle123"])
def test_ackermann_restart_target_is_held_until_carla_reaches_release_speed(actor_id):
    cosim = _make_ackermann_restart_test_cosim(actor_id)
    cosim._ackermann_actor_state = {
        actor_id: {
            "restart_active": True,
            "restart_target_speed": 0.276,
        }
    }

    held_target, _acceleration = cosim._resolve_ackermann_longitudinal_target(
        actor_id,
        {
            "speed": 0.192,
            "sumo_desired_speed": 0.192,
            "feedback_observed_speed": 0.1,
            "acceleration": 1.84,
        },
        current_speed=0.1,
    )
    assert held_target == pytest.approx(0.276)
    assert cosim._ackermann_actor_state[actor_id]["restart_active"] is True

    released_target, _acceleration = cosim._resolve_ackermann_longitudinal_target(
        actor_id,
        {
            "speed": 0.292,
            "sumo_desired_speed": 0.292,
            "feedback_observed_speed": 0.2,
            "acceleration": 1.84,
        },
        current_speed=0.2,
    )
    assert released_target == pytest.approx(0.292)
    assert cosim._ackermann_actor_state[actor_id]["restart_active"] is False
    assert "restart_target_speed" not in cosim._ackermann_actor_state[actor_id]


@pytest.mark.parametrize("actor_id", ["AV", "vehicle123"])
@pytest.mark.parametrize(
    ("sumo_next_speed", "requested_acceleration"),
    [
        (0.0, 1.0),
        (0.05, 0.0),
        (0.05, -1.0),
    ],
)
def test_ackermann_restart_is_cancelled_by_sumo_stop_or_deceleration(
    actor_id,
    sumo_next_speed,
    requested_acceleration,
):
    cosim = _make_ackermann_restart_test_cosim(actor_id)
    cosim._ackermann_actor_state = {
        actor_id: {
            "restart_active": True,
            "restart_target_speed": 0.276,
        }
    }

    target, _acceleration = cosim._resolve_ackermann_longitudinal_target(
        actor_id,
        {
            "speed": sumo_next_speed,
            "sumo_desired_speed": sumo_next_speed,
            "feedback_observed_speed": 0.0,
            "acceleration": requested_acceleration,
        },
        current_speed=0.0,
    )

    assert target == pytest.approx(sumo_next_speed)
    assert cosim._ackermann_actor_state[actor_id]["restart_active"] is False
    assert "restart_target_speed" not in cosim._ackermann_actor_state[actor_id]


def test_ackermann_longitudinal_error_compares_positions_at_t_plus_dt():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    error = CarlaCosim._phase_aligned_longitudinal_error(
        current_rear_axle=(10.0, 20.0, 0.0),
        desired_rear_axle=(10.8, 20.0, 0.0),
        current_velocity=types.SimpleNamespace(x=16.0, y=0.0),
        desired_heading=0.0,
        step_length=0.05,
    )

    assert error == pytest.approx(0.0)


def test_ackermann_longitudinal_error_uses_physical_front_progress_on_curves():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    current = FakeTransform(
        FakeLocation(x=10.0, y=20.0, z=0.0),
        FakeRotation(yaw=0.0),
    )
    # The desired lane yaw differs, but its physical front is exactly one
    # 0.05-second trajectory step ahead of the current physical front.
    desired_yaw = math.radians(30.0)
    front_offset = 2.5
    desired = FakeTransform(
        FakeLocation(
            x=12.5 + 0.5 - math.cos(desired_yaw) * front_offset,
            y=20.0 - math.sin(desired_yaw) * front_offset,
            z=0.0,
        ),
        FakeRotation(yaw=30.0),
    )

    error = CarlaCosim._phase_aligned_front_progress_error(
        current,
        desired,
        front_offset,
        types.SimpleNamespace(x=10.0, y=0.0),
        desired_heading=0.0,
        step_length=0.05,
    )

    assert error == pytest.approx(0.0)


def test_ackermann_feedback_uses_sumo_emergency_decel():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"AV"}
    cosim.ackermann_feedback_all_background_actors = False
    cosim.step_length = 0.05
    cosim.ackermann_tuning = AckermannTuning(max_accel=3.0, max_decel=6.0)
    cosim._ackermann_actor_state = {}

    target, acceleration = cosim._resolve_ackermann_longitudinal_target(
        "AV",
        {
            "speed": 1.0,
            "sumo_desired_speed": 0.0,
            "feedback_observed_speed": 1.0,
            "sumo_emergency_decel": 7.06,
            "acceleration": -7.06,
        },
        current_speed=1.0,
    )

    assert acceleration == pytest.approx(-7.06)
    assert target == pytest.approx(0.0)


def test_ackermann_non_feedback_actor_keeps_sumo_acceleration_for_emergency_brake():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = False
    cosim.ackermann_feedback_actor_ids = set()
    cosim.ackermann_feedback_all_background_actors = False
    cosim.ackermann_tuning = AckermannTuning()
    cosim._ackermann_actor_state = {}

    target, acceleration = cosim._resolve_ackermann_longitudinal_target(
        "AV",
        {"speed": 8.0, "acceleration": -4.13, "sumo_emergency_decel": 9.0},
        current_speed=8.2,
    )

    assert target == pytest.approx(8.0)
    assert acceleration is None
    assert cosim._ackermann_actor_state["AV"]["sumo_requested_acceleration"] == (
        pytest.approx(-4.13)
    )


def test_direct_emergency_brake_engages_and_releases_with_hysteresis():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannEmergencyBrakeTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_emergency_brake_tuning = AckermannEmergencyBrakeTuning(
        engage_decel=4.0,
        release_decel=1.0,
        release_ticks=3,
        stop_speed=0.2,
        min_brake=0.5,
    )
    cosim.ackermann_tuning = AckermannTuning(max_decel=6.0)
    cosim._ackermann_actor_state = {
        "vehicle2322": {
            "steer": 0.3,
            "sumo_requested_acceleration": -4.13,
            "sumo_emergency_decel": 9.0,
        }
    }
    vehicle = types.SimpleNamespace(
        get_control=lambda: types.SimpleNamespace(steer=0.25)
    )

    control = cosim._update_ackermann_emergency_brake(
        "vehicle2322", vehicle, current_speed=8.21
    )
    assert isinstance(control, FakeVehicleControl)
    assert control.throttle == pytest.approx(0.0)
    assert control.brake == pytest.approx(0.5)
    assert control.steer == pytest.approx(0.25)

    state = cosim._ackermann_actor_state["vehicle2322"]
    state["sumo_requested_acceleration"] = -0.5
    assert cosim._update_ackermann_emergency_brake(
        "vehicle2322", vehicle, current_speed=7.0
    )
    assert state["emergency_brake_release_ticks"] == 1
    assert cosim._update_ackermann_emergency_brake(
        "vehicle2322", vehicle, current_speed=6.8
    )
    assert state["emergency_brake_release_ticks"] == 2
    assert (
        cosim._update_ackermann_emergency_brake(
            "vehicle2322", vehicle, current_speed=6.5
        )
        is None
    )
    assert state["control_mode"] == "ackermann"


def test_direct_emergency_brake_releases_at_stop_without_reengaging():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import (
        AckermannEmergencyBrakeTuning,
        AckermannTuning,
    )
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_emergency_brake_tuning = AckermannEmergencyBrakeTuning()
    cosim.ackermann_tuning = AckermannTuning()
    cosim._ackermann_actor_state = {
        "AV": {
            "emergency_brake_active": True,
            "sumo_requested_acceleration": -9.0,
            "sumo_emergency_decel": 9.0,
        }
    }
    vehicle = types.SimpleNamespace(
        get_control=lambda: types.SimpleNamespace(steer=0.0)
    )

    assert cosim._update_ackermann_emergency_brake(
        "AV", vehicle, current_speed=0.1
    ) is None
    assert cosim._update_ackermann_emergency_brake(
        "AV", vehicle, current_speed=0.1
    ) is None


def test_fail_closed_brake_uses_last_sumo_emergency_decel():
    install_fake_carla()
    from terasim_service.utils.carla.ackermann_control import AckermannTuning
    from terasim_service.utils.carla.cosim import CarlaCosim

    controls = []
    ticks = []
    actor = types.SimpleNamespace(
        apply_control=controls.append,
        get_control=lambda: types.SimpleNamespace(steer=0.2),
    )
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_feedback_apply_enabled = True
    cosim.ackermann_feedback_actor_ids = {"AV"}
    cosim.ackermann_feedback_all_background_actors = False
    cosim._ackermann_feedback_actor_index = {"AV": actor}
    cosim._ackermann_actor_state = {"AV": {"steer": 0.2, "sumo_emergency_decel": 7.06}}
    cosim.ackermann_tuning = AckermannTuning(max_decel=6.0)
    cosim.step_length = 0.1
    cosim.args = types.SimpleNamespace(passive_tick=False)
    cosim.world = types.SimpleNamespace(tick=lambda: ticks.append("tick"))

    assert cosim._apply_ackermann_fail_closed_brake("test") == 1
    assert len(controls) == 1
    assert controls[0].throttle == pytest.approx(0.0)
    assert controls[0].brake == pytest.approx(1.0)
    assert controls[0].steer == pytest.approx(0.2)
    assert ticks == ["tick"]


def test_direct_emergency_brake_can_be_batched_as_vehicle_control():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    batch = []
    actor = types.SimpleNamespace(id=42)
    control = FakeVehicleControl(throttle=0.0, brake=0.75)

    CarlaCosim._queue_actor_ackermann_control(
        CarlaCosim.__new__(CarlaCosim),
        actor,
        control,
        batch,
        direct_vehicle_control=True,
    )

    assert batch == [("vehicle_control", 42, control)]


def test_direct_command_failure_stops_before_sumo_step():
    from terasim_service.plugins import cosim_direct as direct_module

    plugin = direct_module.TeraSimCoSimDirectPlugin.__new__(direct_module.TeraSimCoSimDirectPlugin)
    plugin._lock = direct_module.threading.Lock()
    plugin._status = "wait_for_tick"
    plugin._state_json = ""
    plugin._completed_sumo_time = 0.0
    plugin._completed_tick_count = 0
    plugin._pending_commands = [b"invalid"]
    plugin._stop_requested = False
    plugin._tick_requested = direct_module.threading.Event()
    plugin._tick_requested.set()
    plugin._step_done = direct_module.threading.Event()
    plugin.controlled_agents_each_step = set()
    plugin.last_agent_command_failure = {
        "actor_id": "AV",
        "reason": "ackermann_feedback_moveTo_mapping_failed",
    }
    plugin.redis_client = None
    plugin._handle_agent_command = lambda raw: False
    plugin.logger = types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
        critical=lambda *args, **kwargs: None,
    )
    simulator = types.SimpleNamespace(
        running=True,
        env=types.SimpleNamespace(record={}),
    )

    assert plugin.function_before_env_step(simulator, {}) is False
    assert simulator.running is False
    assert simulator.env.record["finish_reason"] == ("ackermann_feedback_moveTo_mapping_failed")
    assert plugin._status == "error"
    assert plugin._step_done.is_set()


def test_background_feedback_failure_can_continue_without_relaxing_av():
    from terasim_service.plugins import cosim as plugin_module

    plugin = plugin_module.TeraSimCoSimPlugin.__new__(plugin_module.TeraSimCoSimPlugin)
    plugin.continue_on_ackermann_feedback_failure = False
    plugin.continue_on_background_ackermann_feedback_failure = True
    plugin.logger = types.SimpleNamespace(warning=lambda *args, **kwargs: None)

    plugin.last_agent_command_failure = {
        "actor_id": "vehicle2311",
        "reason": "ackermann_feedback_moveTo_mapping_failed",
        "ackermann_feedback": True,
    }
    assert plugin._should_continue_after_agent_command_failure() is True

    plugin.last_agent_command_failure = {
        "actor_id": "AV",
        "reason": "ackermann_feedback_moveTo_mapping_failed",
        "ackermann_feedback": True,
    }
    assert plugin._should_continue_after_agent_command_failure() is False

    plugin.last_agent_command_failure = {
        "actor_id": "vehicle2311",
        "reason": "agent_command_exception",
        "ackermann_feedback": False,
    }
    assert plugin._should_continue_after_agent_command_failure() is False


def test_ackermann_control_trace_records_sumo_command_and_carla_response(capsys):
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.ackermann_control_log_records = True
    cosim._ackermann_actor_state = {
        "AV": {
            "sumo_requested_acceleration": -7.06,
            "sumo_emergency_decel": 7.06,
            "restart_active": True,
            "restart_target_speed": 0.276,
            "wheel_base_m": 2.85,
            "rear_axle_local_x_m": -1.4,
            "emergency_brake_active": True,
        }
    }
    cosim.step_length = 0.1
    cosim.terasim_states = {"simulation_time": 12.3}
    cosim.world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=42))
    vehicle = types.SimpleNamespace(
        get_acceleration=lambda: types.SimpleNamespace(x=-6.5, y=0.0, z=0.0),
        get_control=lambda: types.SimpleNamespace(throttle=0.0, brake=0.75, steer=0.1),
    )
    transform = FakeTransform(rotation=FakeRotation(yaw=0.0))

    cosim._record_ackermann_control_trace(
        veh_id="AV",
        veh_info={
            "sumo_desired_speed": 0.0,
            "acceleration": -7.06,
            "feedback_observed_speed": 6.62,
            "lookahead_distance": 3.5,
            "lookahead_heading_change": 0.8,
        },
        vehicle=vehicle,
        current_transform=transform,
        current_speed=6.19,
        target_speed=0.0,
        target_acceleration=-7.06,
        position_error=0.4,
        feedback_unhealthy=False,
        target_behind=False,
        control_values=types.SimpleNamespace(
            raw_steer=0.7,
            clamped_steer=0.6,
            steer=0.55,
            lookahead_local_x=3.0,
            lookahead_local_y=1.0,
            control_point_x=10.0,
            control_point_y=20.0,
        ),
        control_mode="emergency_brake",
        commanded_throttle=0.0,
        commanded_brake=0.78,
        commanded_steer=0.1,
    )

    prefix, payload = capsys.readouterr().out.strip().split(" ", 1)
    record = json.loads(payload)
    assert prefix == "AckermannControlTrace"
    assert record["sumo_requested_acceleration"] == pytest.approx(-7.06)
    assert record["restart_active"] is True
    assert record["restart_target_speed"] == pytest.approx(0.276)
    assert record["ackermann_target_acceleration"] == pytest.approx(-7.06)
    assert record["control_mode"] == "emergency_brake"
    assert record["commanded_throttle"] == pytest.approx(0.0)
    assert record["commanded_brake"] == pytest.approx(0.78)
    assert record["commanded_steer"] == pytest.approx(0.1)
    assert record["emergency_brake_active"] is True
    assert record["carla_speed"] == pytest.approx(6.19)
    assert record["carla_longitudinal_acceleration"] == pytest.approx(-6.5)
    assert record["carla_applied_throttle"] == pytest.approx(0.0)
    assert record["carla_applied_brake"] == pytest.approx(0.75)
    assert record["carla_applied_steer"] == pytest.approx(0.1)
    assert record["lookahead_distance"] == pytest.approx(3.5)
    assert record["lookahead_heading_change"] == pytest.approx(0.8)
    assert record["wheel_base_m"] == pytest.approx(2.85)
    assert record["rear_axle_local_x_m"] == pytest.approx(-1.4)
    assert record["pure_pursuit_raw_steer"] == pytest.approx(0.7)
    assert record["pure_pursuit_clamped_steer"] == pytest.approx(0.6)
    assert record["pure_pursuit_command_steer"] == pytest.approx(0.55)


def test_carla_actor_role_index_is_persistent_and_cleanup_is_batched():
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    actors = [
        types.SimpleNamespace(
            id=1, type_id="vehicle.test", attributes={"role_name": "AV"}, is_alive=True
        ),
        types.SimpleNamespace(
            id=2,
            type_id="vehicle.test",
            attributes={"role_name": "stale"},
            is_alive=True,
        ),
        types.SimpleNamespace(
            id=3,
            type_id="walker.pedestrian.test",
            attributes={"role_name": "VRU_1"},
            is_alive=True,
        ),
    ]
    scans = []
    batches = []
    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim.world = types.SimpleNamespace(
        get_actors=lambda: scans.append("scan") or actors
    )
    cosim.client = types.SimpleNamespace(
        apply_batch_sync=lambda commands, due_tick: batches.append(
            (commands, due_tick)
        )
    )
    cosim.args = types.SimpleNamespace(protected_roles=["AV"])
    cosim._vehicle_actor_index = None
    cosim._pedestrian_actor_index = None
    cosim._pending_actor_index_entries = {}

    first_vehicle, first_pedestrian = cosim._build_actor_role_indexes()
    second_vehicle, second_pedestrian = cosim._build_actor_role_indexes()

    assert scans == ["scan"]
    assert first_vehicle is second_vehicle
    assert first_pedestrian is second_pedestrian
    assert set(first_vehicle) == {"AV", "stale"}
    cosim._cleanup_actors("vehicle", "vehicle.*", {"AV"})
    assert len(batches) == 1
    assert len(batches[0][0]) == 1
    assert batches[0][1] is False
    assert set(first_vehicle) == {"AV"}


def test_traffic_light_static_information_is_cached():
    from terasim_service.plugins.cosim import TeraSimCoSimPlugin

    network_calls = []
    program = types.SimpleNamespace(getParams=lambda: {"offset": "1"})
    tls = types.SimpleNamespace(getPrograms=lambda: {"p0": program})
    plugin = TeraSimCoSimPlugin.__new__(TeraSimCoSimPlugin)
    plugin.traffic_light_information_cache = {}
    plugin.simulator = types.SimpleNamespace(
        sumo_net=types.SimpleNamespace(
            getTLS=lambda traffic_light_id: network_calls.append(traffic_light_id)
            or tls
        )
    )

    first = plugin._get_traffic_light_information("tls_0")
    second = plugin._get_traffic_light_information("tls_0")

    assert first == second
    assert json.loads(first) == {
        "programs": {"p0": {"parameters": {"offset": "1"}}}
    }
    assert network_calls == ["tls_0"]


def test_agent_command_accepts_structured_dict_without_json_roundtrip():
    from terasim_service.plugins.cosim import TeraSimCoSimPlugin
    from terasim_service.utils import AgentCommand

    plugin = TeraSimCoSimPlugin.__new__(TeraSimCoSimPlugin)
    applied = []
    plugin._apply_agent_command = lambda command, parse_elapsed=0.0: applied.append(
        (command, parse_elapsed)
    ) or True

    assert plugin._handle_agent_command(
        {
            "agent_id": "AV",
            "agent_type": "vehicle",
            "command_type": "set_state",
            "data": {"position": [1.0, 2.0]},
        }
    )
    assert isinstance(applied[0][0], AgentCommand)
    assert applied[0][0].agent_id == "AV"
    assert applied[0][1] >= 0.0


def test_inprocess_link_transfers_python_objects_and_tracks_generation():
    import threading

    from terasim_service.plugins.cosim_inprocess import (
        InProcessLink,
        TeraSimCoSimInProcessPlugin,
    )
    from terasim_service.utils import AgentCommand

    plugin = TeraSimCoSimInProcessPlugin.__new__(TeraSimCoSimInProcessPlugin)
    plugin._lock = threading.Lock()
    plugin._status = "wait_for_tick"
    plugin._state = {"simulation_time": 0.0}
    plugin._completed_sumo_time = 0.0
    plugin._completed_tick_count = 0
    plugin._completed_generation = 0
    plugin._requested_generation = 0
    plugin._request_in_flight = False
    plugin._stop_requested = False
    plugin._pending_commands = []
    plugin._tick_requested = threading.Event()
    plugin._step_done = threading.Event()

    link = InProcessLink(plugin, ready_timeout=0.1)
    future = link.tick_async(
        [
            {
                "agent_id": "AV",
                "agent_type": "vehicle",
                "command_type": "set_state",
                "data": {"position": [1.0, 2.0]},
            }
        ]
    )
    assert plugin._tick_requested.is_set()
    assert isinstance(plugin._pending_commands[0], AgentCommand)
    with plugin._lock:
        plugin._state = {"simulation_time": 0.05}
        plugin._completed_sumo_time = 0.05
        plugin._completed_tick_count = 1
        plugin._completed_generation = 1
        plugin._request_in_flight = False
        plugin._status = "ticked"
    plugin._step_done.set()

    response = future.result(timeout=0.1)
    assert response.state == {"simulation_time": 0.05}
    assert response.completed_tick_count == 1


def test_inprocess_state_conversion_returns_nested_plain_dicts():
    from terasim_service.plugins.cosim_inprocess import TeraSimCoSimInProcessPlugin
    from terasim_service.utils import AgentStateSimplified, SimulationState, SUMOSignal

    state = SimulationState(
        simulation_time=1.25,
        agent_count={"vehicle": 1},
        agent_details={
            "vehicle": {"AV": AgentStateSimplified(x=1.0, speed=2.0)}
        },
        traffic_light_details={"tls": SUMOSignal(tls="Gr")},
    )

    plain = TeraSimCoSimInProcessPlugin._simulation_state_to_plain_dict(state)

    assert type(plain) is dict
    assert type(plain["agent_details"]["vehicle"]["AV"]) is dict
    assert plain["agent_details"]["vehicle"]["AV"]["speed"] == pytest.approx(2.0)
    assert type(plain["traffic_light_details"]["tls"]) is dict
    assert plain["traffic_light_details"]["tls"]["tls"] == "Gr"


class FakeDiagnosticActor:
    def __init__(self, actor_id, role_name, x=0.0, speed=0.0):
        self.id = actor_id
        self.type_id = "vehicle.test"
        self.attributes = {"role_name": role_name}
        self._transform = FakeTransform(FakeLocation(x=x), FakeRotation())
        self._velocity = FakeVector3D(x=speed)

    def get_transform(self):
        return self._transform

    def get_velocity(self):
        return self._velocity

    def get_acceleration(self):
        return FakeVector3D()

    def get_angular_velocity(self):
        return FakeVector3D()

    def get_control(self):
        return types.SimpleNamespace(
            throttle=0.0,
            steer=0.0,
            brake=0.0,
            hand_brake=False,
            reverse=False,
            gear=1,
        )


def test_carla_collision_diagnostics_deduplicates_sensor_sides_and_counts_episodes(
    tmp_path,
):
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._diagnostic_lock = __import__("threading").Lock()
    cosim.collision_sensor_enabled = True
    cosim.collision_log_path = str(tmp_path / "collisions.jsonl")
    cosim.collision_summary_path = str(tmp_path / "collision_summary.json")
    cosim.collision_episode_gap_frames = 10
    cosim._collision_seen_frame_pairs = set()
    cosim._collision_last_pair_frame = {}
    cosim._collision_raw_event_count = 0
    cosim._collision_unique_frame_count = 0
    cosim._collision_episode_count = 0
    cosim._collision_episode_counts_by_pair = {}
    cosim._initialization_failure_counts = {}

    first = FakeDiagnosticActor(11, "vehicle11", speed=4.0)
    second = FakeDiagnosticActor(22, "vehicle22", speed=3.0)
    event_from_first = types.SimpleNamespace(
        frame=100,
        timestamp=5.0,
        other_actor=second,
        normal_impulse=FakeVector3D(x=10.0),
    )
    event_from_second = types.SimpleNamespace(
        frame=100,
        timestamp=5.0,
        other_actor=first,
        normal_impulse=FakeVector3D(x=-10.0),
    )
    repeated_contact = types.SimpleNamespace(
        frame=105,
        timestamp=5.25,
        other_actor=second,
        normal_impulse=FakeVector3D(x=1.0),
    )
    new_contact = types.SimpleNamespace(
        frame=116,
        timestamp=5.8,
        other_actor=second,
        normal_impulse=FakeVector3D(x=2.0),
    )

    cosim._on_collision_event("vehicle11", first, event_from_first)
    cosim._on_collision_event("vehicle22", second, event_from_second)
    cosim._on_collision_event("vehicle11", first, repeated_contact)
    cosim._on_collision_event("vehicle11", first, new_contact)
    cosim._write_collision_summary()

    summary = json.loads((tmp_path / "collision_summary.json").read_text())
    assert summary["raw_sensor_events"] == 4
    assert summary["unique_frame_pairs"] == 3
    assert summary["contact_episodes"] == 2
    assert summary["episodes_by_pair"] == {"11:22": 2}
    records = [
        json.loads(line)
        for line in (tmp_path / "collisions.jsonl").read_text().splitlines()
    ]
    assert records[1]["duplicate_frame_pair"] is True
    assert records[3]["new_episode"] is True


def test_carla_initialization_failure_diagnostic_captures_actual_state(tmp_path):
    install_fake_carla()
    from terasim_service.utils.carla.cosim import CarlaCosim

    cosim = CarlaCosim.__new__(CarlaCosim)
    cosim._diagnostic_lock = __import__("threading").Lock()
    cosim.initialization_diagnostics_enabled = True
    cosim.initialization_log_path = str(tmp_path / "initialization.jsonl")
    cosim._initialization_failure_counts = {}
    cosim.world = types.SimpleNamespace(
        get_snapshot=lambda: types.SimpleNamespace(frame=123)
    )
    actor = FakeDiagnosticActor(31, "vehicle31", x=8.0, speed=19.0)

    cosim._record_initialization_diagnostic(
        "failure",
        actor,
        "vehicle31",
        expected_transform=FakeTransform(FakeLocation(x=7.0)),
        expected_speed=4.0,
        reason="speed_error=15.000m/s",
        attempt=2,
    )

    record = json.loads((tmp_path / "initialization.jsonl").read_text())
    assert record["carla_frame"] == 123
    assert record["reason"] == "speed_error=15.000m/s"
    assert record["actual"]["velocity"]["x"] == pytest.approx(19.0)
    assert record["expected_speed"] == pytest.approx(4.0)
    assert cosim._initialization_failure_counts == {"speed_error": 1}
