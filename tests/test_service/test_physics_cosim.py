import unittest
from threading import Event, Lock
from types import SimpleNamespace
from unittest.mock import patch

from terasim_service.plugins import cosim_inprocess
from terasim_service.plugins.cosim_inprocess import TeraSimCoSimInProcessPlugin
from terasim_service.utils import AgentCommand
from terasim_service.utils import base as service_base
from terasim_service.utils.carla.ackermann_control import (
    AckermannTuning,
    compute_ackermann_control_values,
    compute_direct_brake_value,
)
from terasim_service.utils.carla.cosim import CarlaCosim
from terasim_service.utils.sumo_lane_geometry import (
    build_external_state_lateral_action_lookahead,
    compile_lane_shapes,
)


class _FakeVehicle:
    def __init__(self, calls):
        self.calls = calls
        self.position = (10.0, 1.0)
        self.angle = 90.0
        self.speed = 4.0
        self.acceleration = 0.5

    def getLaneID(self, actor_id):
        return "edge_0_0"

    def getRoute(self, actor_id):
        return ("edge_0", "edge_1")

    def moveToXYImmediate(
        self,
        actor_id,
        edge_id,
        lane_index,
        x,
        y,
        angle,
        keep_route,
        threshold,
        strict_lane,
    ):
        self.calls.append(("move", actor_id, edge_id, lane_index, strict_lane))
        self.position = (x, y)
        self.angle = angle

    def setSpeed(self, actor_id, speed):
        self.calls.append(("setSpeed", speed))

    def setPreviousSpeed(self, actor_id, speed, acceleration):
        self.calls.append(("setPreviousSpeed", speed, acceleration))
        self.speed = speed
        self.acceleration = acceleration

    def getPosition(self, actor_id):
        return self.position

    def getAngle(self, actor_id):
        return self.angle

    def getSpeed(self, actor_id):
        return self.speed

    def getAcceleration(self, actor_id):
        return self.acceleration

    def getLanePosition(self, actor_id):
        return self.position[0]


class _FakeLane:
    @staticmethod
    def getEdgeID(lane_id):
        return "edge_0"

    @staticmethod
    def getShape(lane_id):
        return ((0.0, 1.0), (100.0, 1.0))

    @staticmethod
    def getLength(lane_id):
        return 100.0


class PhysicalCosimTest(unittest.TestCase):
    def test_ackermann_steer_is_rate_limited_toward_lookahead(self):
        values = compute_ackermann_control_values(
            current_x=0.0,
            current_y=0.0,
            yaw_degrees=0.0,
            current_speed=5.0,
            desired_x=0.25,
            desired_y=0.0,
            lookahead_x=7.0,
            lookahead_y=2.0,
            desired_speed=5.5,
            previous_steer=0.0,
            dt=0.05,
            tuning=AckermannTuning(max_steer_rate_rad_s=0.6),
        )
        self.assertGreater(values.steer, 0.0)
        self.assertLessEqual(values.steer, 0.03 + 1e-12)
        self.assertGreater(values.lookahead_local_x, 0.0)

    def test_direct_brake_mapping_has_minimum_and_saturates(self):
        self.assertAlmostEqual(
            compute_direct_brake_value(-1.0, 8.0, 0.5), 0.5
        )
        self.assertAlmostEqual(
            compute_direct_brake_value(-20.0, 8.0, 0.5), 1.0
        )

    def test_lateral_action_lookahead_uses_sumo_lateral_velocity(self):
        path = compile_lane_shapes([((0.0, 0.0), (20.0, 0.0))])
        result = build_external_state_lateral_action_lookahead(
            path,
            (0.0, 0.0),
            7.0,
            target_lane_shape=((0.0, 3.5), (20.0, 3.5)),
            lateral_speed=1.0,
            desired_speed=5.0,
            maneuver_active=True,
            z=0.0,
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["mode"], "sumo_lateral_velocity")
        self.assertAlmostEqual(result["lookahead"][0], 7.0)
        self.assertAlmostEqual(result["lookahead"][1], 1.4)
        self.assertAlmostEqual(result["lateral_displacement"], 1.4)

    def test_lateral_action_lookahead_limits_preview_change_per_step(self):
        path = compile_lane_shapes([((0.0, 0.0), (20.0, 0.0))])
        result = build_external_state_lateral_action_lookahead(
            path,
            (0.0, 0.0),
            7.0,
            target_lane_shape=((0.0, 3.5), (20.0, 3.5)),
            lateral_speed=2.0,
            desired_speed=5.0,
            maneuver_active=True,
            previous_lateral_displacement=0.2,
            max_lateral_displacement_change=0.05,
        )
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["lateral_displacement"], 0.25)

    def test_phase_a_immediate_move_does_not_advance_sumo_time(self):
        calls = []
        fake_vehicle = _FakeVehicle(calls)
        fake_traci = SimpleNamespace(
            vehicle=fake_vehicle,
            lane=_FakeLane(),
            simulation=SimpleNamespace(getTime=lambda: 12.5),
        )

        plugin = object.__new__(TeraSimCoSimInProcessPlugin)
        plugin.physics_lane_geometry_cache = {}
        plugin.physics_match_threshold = 8.0
        plugin.physics_strict_lane_hint = True
        plugin.physics_validate_external_state = True
        plugin.physics_position_tolerance = 0.001
        plugin.physics_feedback_observations = {}

        command = AgentCommand(
            agent_id="vehicle0",
            agent_type="vehicle",
            command_type="set_state",
            data={
                "position": [10.0, 1.0],
                "z": 0.0,
                "sumo_angle": 90.0,
                "speed": 4.0,
                "acceleration": 0.5,
                "source_carla_frame": 123,
            },
        )
        with patch.object(cosim_inprocess, "traci", fake_traci):
            self.assertTrue(
                plugin._apply_physics_external_state(command, 10.0, 1.0)
            )
        self.assertEqual(
            [call[0] for call in calls],
            ["move", "setSpeed", "setPreviousSpeed"],
        )
        observation = plugin.physics_feedback_observations["vehicle0"]
        self.assertAlmostEqual(observation["phase_a_time"], 12.5)
        self.assertEqual(observation["source_carla_frame"], 123)

    def test_create_simulator_passes_optional_step_length(self):
        config = {
            "input": {
                "sumo_net_file": "net.xml",
                "sumo_config_file": "sumo.sumocfg",
            },
            "simulator": {
                "parameters": {
                    "num_tries": 10,
                    "gui_flag": False,
                    "realtime_flag": False,
                    "sumo_output_file_types": [],
                    "sumo_seed": 42,
                    "step_length": 0.05,
                }
            },
        }
        with patch.object(service_base, "Simulator") as simulator:
            service_base.create_simulator(config, "output")
        self.assertEqual(simulator.call_args.kwargs["step_length"], 0.05)

    def test_phase_a_observations_are_cleared_at_each_tick_start(self):
        plugin = object.__new__(TeraSimCoSimInProcessPlugin)
        plugin._stop_requested = False
        plugin._tick_requested = Event()
        plugin._tick_requested.set()
        plugin._pending_commands = []
        plugin._lock = Lock()
        plugin.controlled_agents_each_step = set()
        plugin.physics_feedback_observations = {
            "filtered_vehicle": {"phase_a_time": 1.0}
        }
        plugin._set_status = lambda status: None
        plugin.logger = SimpleNamespace(debug=lambda message: None)

        self.assertTrue(plugin.function_before_env_step(None, None))
        self.assertEqual(plugin.physics_feedback_observations, {})

    def test_physics_initialization_waits_one_completed_carla_frame(self):
        calls = []

        class FakeActor:
            def set_simulate_physics(self, enabled):
                calls.append(("physics", enabled))

            def set_transform(self, transform):
                calls.append(("transform", transform))

            def set_target_velocity(self, velocity):
                calls.append(("velocity", velocity.x, velocity.y, velocity.z))

        transform = SimpleNamespace(
            get_forward_vector=lambda: SimpleNamespace(x=1.0, y=0.0)
        )
        actor = FakeActor()
        cosim = object.__new__(CarlaCosim)
        cosim._ackermann_actor_state = {}
        cosim._last_world_frame = 100
        cosim._configure_ackermann_actor = lambda configured_actor: calls.append(
            ("configure", configured_actor)
        )

        self.assertFalse(
            cosim._initialize_ackermann_actor(actor, "vehicle0", transform, 4.0)
        )
        self.assertEqual([call[0] for call in calls], ["physics", "transform"])

        self.assertFalse(
            cosim._initialize_ackermann_actor(actor, "vehicle0", transform, 4.0)
        )
        self.assertEqual([call[0] for call in calls], ["physics", "transform"])

        cosim._last_world_frame = 101
        self.assertTrue(
            cosim._initialize_ackermann_actor(actor, "vehicle0", transform, 4.0)
        )
        self.assertEqual(
            [call[0] for call in calls],
            ["physics", "transform", "physics", "velocity", "configure"],
        )
        self.assertTrue(cosim._ackermann_actor_state["vehicle0"]["initialized"])

    def test_physics_selection_never_takes_autoware_ego(self):
        cosim = object.__new__(CarlaCosim)
        cosim.ackermann_feedback_apply_enabled = True
        cosim.ackermann_feedback_actor_ids = {"*"}
        self.assertTrue(cosim._is_ackermann_feedback_actor("vehicle0"))
        self.assertFalse(cosim._is_ackermann_feedback_actor("AV"))

        plugin = object.__new__(TeraSimCoSimInProcessPlugin)
        plugin.physics_external_state_enabled = True
        plugin.physics_feedback_actor_ids = {"*"}
        self.assertTrue(plugin._is_physics_feedback_actor("vehicle0"))
        self.assertFalse(plugin._is_physics_feedback_actor("AV"))


if __name__ == "__main__":
    unittest.main()
