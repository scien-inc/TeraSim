import time
import json
from contextlib import contextmanager
import math
import os
import re
import carla
import random
import xml.etree.ElementTree as ET
import yaml
import statistics
import threading

from .ackermann_control import (
    AckermannControllerTuning,
    AckermannEmergencyBrakeTuning,
    AckermannTuning,
    compute_ackermann_control_values,
    compute_direct_brake_value,
    horizontal_speed,
)
from .tools import (
    carla_to_sumo,
    create_bike_blueprint,
    create_bikeandmotor_blueprint,
    create_motor_blueprint,
    create_pedestrian_blueprint,
    create_police_car_blueprint,
    create_vehicle_blueprint,
    destroy_all_actors,
    draw_text,
    get_actor_id_from_attribute,
    log_spawn_actor_failure,
    sumo_point_to_carla,
    sumo_to_carla,
    spawn_actor,
)
from ..service import (
    control_agent,
    control_agents_batch,
    start_terasim,
    stop_terasim,
    tick_terasim,
    get_terasim_status,
    get_terasim_states,
)

AV_SUMO_ID = "AV"
SUMO_CARLA_TLS_LINK_PREFIX = "linkSignalID:"
VEHICLE_CONTROL_MODE_TELEPORT = "teleport"
VEHICLE_CONTROL_MODE_ACKERMANN_PHYSICS = "ackermann_physics"
VEHICLE_CONTROL_MODES = {
    VEHICLE_CONTROL_MODE_TELEPORT,
    VEHICLE_CONTROL_MODE_ACKERMANN_PHYSICS,
}
ACKERMANN_FEEDBACK_MODE_OFF = "off"
ACKERMANN_FEEDBACK_MODE_SHADOW = "shadow"
ACKERMANN_FEEDBACK_MODE_APPLY = "apply"
ACKERMANN_FEEDBACK_MODES = {
    ACKERMANN_FEEDBACK_MODE_OFF,
    ACKERMANN_FEEDBACK_MODE_SHADOW,
    ACKERMANN_FEEDBACK_MODE_APPLY,
}


def _env_float(name, default):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError:
        print(f"Warning: invalid {name}={value!r}; using {default}.", flush=True)
        return default


def _env_int(name, default):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    try:
        return int(value)
    except ValueError:
        print(f"Warning: invalid {name}={value!r}; using {default}.", flush=True)
        return default


def _env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.lower() in {"1", "true", "yes", "on"}


class CarlaCosim(object):
    def __init__(self, args):
        self.args = args
        # Keep CARLA blueprint selection independent from TeraSim/NDE's global
        # RNG. This is required when both loops share one Python process.
        self._random = random.Random()

        carla_random_seed = os.environ.get("CARLA_COSIM_RANDOM_SEED", "").strip()
        if carla_random_seed:
            try:
                self.carla_random_seed = int(carla_random_seed)
            except ValueError as exc:
                raise ValueError(
                    "CARLA_COSIM_RANDOM_SEED must be an integer, "
                    f"got {carla_random_seed!r}"
                ) from exc
            self._random.seed(self.carla_random_seed)
            print(f"CARLA co-sim random seed: {self.carla_random_seed}", flush=True)

        self.client = carla.Client(args.carla_host, args.carla_port)
        self.client.set_timeout(getattr(args, 'carla_timeout', 10.0))

        self.world = self.client.get_world()
        if args.map_name:
            print(f"Loading map {args.map_name}")
            try:
                self.world = self.client.load_world(args.map_name)
            except:
                print(f"Map {args.map_name} not found. Loading default map.")
        else:
            print("No map name provided. Loading default map.")

        self.traffic_lights = self.world.get_actors().filter("traffic.traffic_light")
        for traffic_light in self.traffic_lights:
            traffic_light.set_state(carla.TrafficLightState.Off)
            traffic_light.freeze(True)

        self.control_av = args.control_av
        self.initialize_av = False
        self.av_shape = []
        self.async_mode = args.async_mode
        self.step_length = args.step_length
        self._spawn_failures = {}
        self._missing_angle_warnings = set()
        self._invalid_location_warnings = set()
        # CARLA actors mirrored from SUMO are indexed by role_name for their
        # entire lifetime. Rebuilding these dictionaries with
        # world.get_actors() every tick is expensive on dense maps.
        self._vehicle_actor_index = None
        self._pedestrian_actor_index = None
        self._pending_actor_index_entries = {}
        self.collision_sensor_enabled = _env_bool(
            "CARLA_COSIM_COLLISION_SENSOR_ENABLED", False
        )
        self.collision_log_path = os.environ.get(
            "CARLA_COSIM_COLLISION_LOG", "/app/outputs/carla_collision_events.jsonl"
        ).strip()
        self.collision_summary_path = os.environ.get(
            "CARLA_COSIM_COLLISION_SUMMARY", "/app/outputs/carla_collision_summary.json"
        ).strip()
        self.collision_episode_gap_frames = max(
            1, _env_int("CARLA_COSIM_COLLISION_EPISODE_GAP_FRAMES", 10)
        )
        self.initialization_diagnostics_enabled = _env_bool(
            "CARLA_COSIM_INITIALIZATION_DIAGNOSTICS_ENABLED", False
        )
        self.initialization_log_path = os.environ.get(
            "CARLA_COSIM_INITIALIZATION_LOG",
            "/app/outputs/carla_physics_initialization.jsonl",
        ).strip()
        self._diagnostic_lock = threading.Lock()
        self._collision_sensors = {}
        self._collision_seen_frame_pairs = set()
        self._collision_last_pair_frame = {}
        self._collision_raw_event_count = 0
        self._collision_unique_frame_count = 0
        self._collision_episode_count = 0
        self._collision_episode_counts_by_pair = {}
        self._initialization_failure_counts = {}
        if self.collision_sensor_enabled:
            self._reset_diagnostic_path(self.collision_log_path)
            self._reset_diagnostic_path(self.collision_summary_path)
            print(
                "CARLA collision sensors enabled: "
                f"events={self.collision_log_path} summary={self.collision_summary_path}.",
                flush=True,
            )
        if self.initialization_diagnostics_enabled:
            self._reset_diagnostic_path(self.initialization_log_path)
            print(
                f"CARLA physics initialization diagnostics: {self.initialization_log_path}.",
                flush=True,
            )
        requested_vehicle_control_mode = (
            getattr(args, "vehicle_control_mode", None)
            or os.environ.get("CARLA_COSIM_VEHICLE_CONTROL_MODE")
            or VEHICLE_CONTROL_MODE_TELEPORT
        )
        self.vehicle_control_mode = str(requested_vehicle_control_mode).strip().lower()
        if self.vehicle_control_mode not in VEHICLE_CONTROL_MODES:
            print(
                f"Warning: invalid vehicle control mode {requested_vehicle_control_mode!r}; "
                f"using {VEHICLE_CONTROL_MODE_TELEPORT}.",
                flush=True,
            )
            self.vehicle_control_mode = VEHICLE_CONTROL_MODE_TELEPORT
        self.ackermann_physics_enabled = (
            self.vehicle_control_mode == VEHICLE_CONTROL_MODE_ACKERMANN_PHYSICS
        )
        requested_feedback_mode = os.environ.get(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_MODE", ACKERMANN_FEEDBACK_MODE_OFF
        )
        self.ackermann_feedback_mode = str(requested_feedback_mode).strip().lower()
        if self.ackermann_feedback_mode not in ACKERMANN_FEEDBACK_MODES:
            raise ValueError(
                "CARLA_COSIM_ACKERMANN_FEEDBACK_MODE must be one of "
                f"{sorted(ACKERMANN_FEEDBACK_MODES)}, got {requested_feedback_mode!r}"
            )
        feedback_actor_value = os.environ.get("CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS", "")
        self.ackermann_feedback_actor_ids = {
            actor_id.strip() for actor_id in feedback_actor_value.split(",") if actor_id.strip()
        }
        self.ackermann_feedback_all_background_actors = "*" in self.ackermann_feedback_actor_ids
        if self.ackermann_feedback_mode != ACKERMANN_FEEDBACK_MODE_OFF:
            if not self.ackermann_physics_enabled:
                raise ValueError(
                    "Ackermann feedback requires vehicle_control_mode=ackermann_physics"
                )
            if self.async_mode:
                raise ValueError("Ackermann feedback requires synchronous CARLA mode")
            if getattr(args, "passive_tick", False):
                raise ValueError("Ackermann feedback is incompatible with passive_tick")
            if self.control_av:
                raise ValueError(
                    "Ackermann feedback and control_av cannot own the AV at the same time"
                )
        self.ackermann_feedback_apply_enabled = (
            self.ackermann_feedback_mode == ACKERMANN_FEEDBACK_MODE_APPLY
            and bool(self.ackermann_feedback_actor_ids)
        )
        self.ackermann_feedback_shadow_enabled = (
            self.ackermann_feedback_mode == ACKERMANN_FEEDBACK_MODE_SHADOW
            and bool(self.ackermann_feedback_actor_ids)
        )
        self._ackermann_feedback_state = {}
        self.ackermann_feedback_log_records = _env_bool(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_LOG_RECORDS", True
        )
        self.ackermann_control_log_records = _env_bool(
            "CARLA_COSIM_ACKERMANN_CONTROL_LOG_RECORDS", False
        )
        control_log_actor_value = os.environ.get(
            "CARLA_COSIM_ACKERMANN_CONTROL_LOG_ACTORS", ""
        )
        self.ackermann_control_log_actor_ids = {
            actor_id.strip()
            for actor_id in control_log_actor_value.split(",")
            if actor_id.strip()
        }
        self._ackermann_feedback_actor_index = {}
        self._ackermann_feedback_candidate_actor_ids = set()
        self._ackermann_actor_state = {}
        self._initial_terasim_state_pending = True
        self._last_completed_terasim_tick_count = None
        self.terasim_states = {}
        self.ackermann_feedback_ack_max_frame_lag = max(
            0, _env_int("CARLA_COSIM_ACKERMANN_FEEDBACK_ACK_MAX_FRAME_LAG", 2)
        )
        self.ackermann_feedback_ack_failure_limit = max(
            1, _env_int("CARLA_COSIM_ACKERMANN_FEEDBACK_ACK_FAILURE_LIMIT", 3)
        )
        self.ackermann_tuning = AckermannTuning(
            wheel_base=max(0.1, _env_float("CARLA_COSIM_ACKERMANN_WHEEL_BASE", 2.8)),
            max_steer_rad=max(0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_STEER_RAD", 0.6)),
            max_steer_rate_rad_s=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_STEER_RATE_RAD_S", 0.6)
            ),
            position_speed_gain=max(
                0.0,
                _env_float("CARLA_COSIM_ACKERMANN_POSITION_SPEED_GAIN", 1.0),
            ),
            kp_speed=_env_float("CARLA_COSIM_ACKERMANN_KP_SPEED", 0.8),
            kp_position=_env_float("CARLA_COSIM_ACKERMANN_KP_POSITION", 0.15),
            max_accel=max(0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_ACCEL", 3.0)),
            max_decel=max(0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_DECEL", 6.0)),
        )
        self.ackermann_controller_tuning = AckermannControllerTuning(
            speed_kp=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_CONTROLLER_SPEED_KP", 1.0)
            ),
            speed_ki=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_CONTROLLER_SPEED_KI", 0.0)
            ),
            speed_kd=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_CONTROLLER_SPEED_KD", 0.0)
            ),
            accel_kp=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_CONTROLLER_ACCEL_KP", 0.05)
            ),
            accel_ki=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_CONTROLLER_ACCEL_KI", 0.0)
            ),
            accel_kd=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_CONTROLLER_ACCEL_KD", 0.0)
            ),
        )
        emergency_brake_engage_decel = max(
            0.0,
            _env_float("CARLA_COSIM_ACKERMANN_EMERGENCY_BRAKE_ENGAGE_DECEL", 4.0),
        )
        self.ackermann_emergency_brake_tuning = AckermannEmergencyBrakeTuning(
            enabled=_env_bool(
                "CARLA_COSIM_ACKERMANN_EMERGENCY_BRAKE_ENABLED", True
            ),
            engage_decel=emergency_brake_engage_decel,
            release_decel=min(
                emergency_brake_engage_decel,
                max(
                    0.0,
                    _env_float(
                        "CARLA_COSIM_ACKERMANN_EMERGENCY_BRAKE_RELEASE_DECEL",
                        1.0,
                    ),
                ),
            ),
            release_ticks=max(
                1,
                _env_int(
                    "CARLA_COSIM_ACKERMANN_EMERGENCY_BRAKE_RELEASE_TICKS", 3
                ),
            ),
            stop_speed=max(
                0.0,
                _env_float(
                    "CARLA_COSIM_ACKERMANN_EMERGENCY_BRAKE_STOP_SPEED", 0.2
                ),
            ),
            min_brake=min(
                1.0,
                max(
                    0.0,
                    _env_float(
                        "CARLA_COSIM_ACKERMANN_EMERGENCY_BRAKE_MIN_BRAKE", 0.5
                    ),
                ),
            ),
        )
        self.ackermann_warn_error_m = max(
            0.0, _env_float("CARLA_COSIM_ACKERMANN_WARN_ERROR_M", 3.0)
        )
        self.ackermann_snap_error_m = max(
            self.ackermann_warn_error_m,
            _env_float("CARLA_COSIM_ACKERMANN_SNAP_ERROR_M", 8.0),
        )
        self.ackermann_warning_interval = max(
            0.0, _env_float("CARLA_COSIM_ACKERMANN_WARNING_INTERVAL", 2.0)
        )
        if self.ackermann_feedback_mode != ACKERMANN_FEEDBACK_MODE_OFF:
            print(
                "CARLA-to-TeraSim Ackermann feedback configured: "
                f"mode={self.ackermann_feedback_mode} "
                f"actors={sorted(self.ackermann_feedback_actor_ids)!r}.",
                flush=True,
            )
        if self.ackermann_physics_enabled:
            print("CARLA co-sim Ackermann physics vehicle control enabled.", flush=True)
            emergency_tuning = self.ackermann_emergency_brake_tuning
            print(
                "CARLA co-sim direct emergency brake "
                f"enabled={emergency_tuning.enabled} "
                f"engage=-{emergency_tuning.engage_decel:.3g}m/s^2 "
                f"release=-{emergency_tuning.release_decel:.3g}m/s^2 "
                f"releaseTicks={emergency_tuning.release_ticks} "
                f"stopSpeed={emergency_tuning.stop_speed:.3g}m/s "
                f"minBrake={emergency_tuning.min_brake:.3g}.",
                flush=True,
            )
        self.use_lane_relative_position = _env_bool(
            "CARLA_COSIM_USE_LANE_RELATIVE_POSITION",
            bool(getattr(args, "use_lane_relative_position", False)),
        )
        if self.use_lane_relative_position:
            print("CARLA co-sim lane-relative reconstructed positions enabled.", flush=True)
        self.spawn_failure_backoff_seconds = max(
            0.0,
            _env_float("CARLA_COSIM_SPAWN_FAILURE_BACKOFF_SECONDS", 5.0),
        )
        self.spawn_failure_backoff_max_seconds = max(
            self.spawn_failure_backoff_seconds,
            _env_float("CARLA_COSIM_SPAWN_FAILURE_BACKOFF_MAX_SECONDS", 30.0),
        )
        self.spawn_max_attempts = max(
            1,
            _env_int("CARLA_COSIM_SPAWN_MAX_ATTEMPTS", 3),
        )
        print(
            f"CARLA co-sim spawn attempt limit: {self.spawn_max_attempts}.",
            flush=True,
        )
        self.batch_transform_enabled = _env_bool("CARLA_COSIM_BATCH_TRANSFORM", False)
        if self.batch_transform_enabled:
            print("CARLA co-sim ApplyTransform batching enabled.", flush=True)
        self.batch_spawn_enabled = _env_bool("CARLA_COSIM_BATCH_SPAWN", False)
        if self.batch_spawn_enabled:
            print("CARLA co-sim SpawnActor batching enabled.", flush=True)
        self.spawn_z_clearance = max(
            0.0,
            _env_float("CARLA_COSIM_SPAWN_Z_CLEARANCE", self.SPAWN_Z_CLEARANCE),
        )
        print(
            f"CARLA co-sim spawn Z clearance: {self.spawn_z_clearance:.1f}m.",
            flush=True,
        )
        self.actor_filter_enabled = _env_bool("CARLA_COSIM_ACTOR_FILTER", False)
        self.actor_filter_center_id = (
            os.environ.get("CARLA_COSIM_ACTOR_FILTER_CENTER_ID") or AV_SUMO_ID
        )
        self.actor_filter_radius = max(
            0.0,
            _env_float("CARLA_COSIM_ACTOR_FILTER_RADIUS", 300.0),
        )
        self.actor_filter_hysteresis = max(
            0.0,
            _env_float("CARLA_COSIM_ACTOR_FILTER_HYSTERESIS", 20.0),
        )
        self._actor_filter_active_vehicle_ids = set()
        self._actor_filter_missing_center_warned = False
        if self.actor_filter_enabled:
            print(
                "CARLA co-sim actor radius filter enabled: "
                f"center={self.actor_filter_center_id} "
                f"enterRadius={self.actor_filter_radius:.1f}m "
                f"exitRadius={self.actor_filter_radius + self.actor_filter_hysteresis:.1f}m.",
                flush=True,
            )

        self.physics_radius = max(
            0.0, _env_float("CARLA_COSIM_PHYSICS_RADIUS", 0.0)
        )
        self.physics_radius_enabled = self.physics_radius > 0.0
        self.physics_radius_center_id = (
            os.environ.get("CARLA_COSIM_PHYSICS_RADIUS_CENTER_ID") or AV_SUMO_ID
        )
        self.physics_radius_hysteresis = max(
            0.0, _env_float("CARLA_COSIM_PHYSICS_RADIUS_HYSTERESIS", 10.0)
        )
        self._physics_active_vehicle_ids = set()
        if getattr(self, "physics_radius_enabled", False):
            print(
                "CARLA co-sim physics radius enabled: "
                f"center={self.physics_radius_center_id} "
                f"enterRadius={self.physics_radius:.1f}m "
                f"exitRadius={self.physics_radius + self.physics_radius_hysteresis:.1f}m.",
                flush=True,
            )

        self.profile_steps_enabled = _env_bool(
            "CARLA_COSIM_PROFILE_STEPS", _env_bool("COSIM_PROFILE_STEPS", False)
        )
        self.profile_jsonl_path = os.environ.get("CARLA_COSIM_PROFILE_JSONL", "").strip()
        self.profile_warmup_steps = max(
            0, _env_int("CARLA_COSIM_PROFILE_WARMUP_STEPS", 0)
        )
        self._profile_step_count = 0
        self._current_step_profile = None
        self._pending_terasim_roundtrip = None

        self.vehicle_blueprints = create_vehicle_blueprint(self.world)
        self.motor_blueprints = create_motor_blueprint(self.world)
        self.pedestrian_blueprints = create_pedestrian_blueprint(self.world)
        self.police_car_blueprints = create_police_car_blueprint(self.world)
        self.bike_blueprints = create_bike_blueprint(self.world)
        self.bikeandmotor_blueprints = create_bikeandmotor_blueprint(self.world)

        # self.sync_cosim_construction_zone_to_carla()

        # start / connect to TeraSim
        self.direct_link = None
        self._direct_tick_future = None
        self._direct_prev_state = None
        inprocess_link = getattr(args, "inprocess_link", None)
        direct_addr = getattr(args, "direct_addr", None)
        if inprocess_link is not None:
            self.direct_link = inprocess_link
            self._direct_prev_state = self._decode_link_state(
                self.direct_link.get_state()
            )
            self.terasim = {"simulation_id": "inprocess"}
            print("TeraSim in-process link ready.", flush=True)
        elif direct_addr:
            # Direct (gRPC) mode: no Redis/FastAPI. The TeraSim runner
            # (terasim_service.run_direct) already owns the simulation; connect
            # and wait until it reaches wait_for_tick.
            from .direct_link import DirectLink, parse_state_json

            self.direct_link = DirectLink(direct_addr)
            # Seed the render pipeline with the initial (post-warmup) state so
            # the first tick behaves like the polling path (AV shape init etc.).
            self._direct_prev_state = parse_state_json(
                self.direct_link.get_state().state_json
            )
            self.terasim = {"simulation_id": "direct"}
        else:
            terasim_init_command = {
                "config_file": args.terasim_config,
                "auto_run": False,
            }
            self.terasim = start_terasim(args.terasim_host, args.terasim_port, terasim_init_command)
            while True:
                terasim_status = get_terasim_status(args.terasim_host, args.terasim_port, self.terasim["simulation_id"])
                if terasim_status.get("status", None) == "wait_for_tick":
                    break
                time.sleep(0.1)

        # Auto-calibrate SUMO-CARLA coordinate transformation
        self.sumo_carla_offset = [0.0, 0.0]
        self._coord_transformer = None
        self._sumo_net_offset = [0.0, 0.0]
        self._xodr_origin_utm = [0.0, 0.0]
        net_file = self._get_net_file_from_config(args.terasim_config)
        if net_file:
            result = self._calibrate_sumo_carla_offset(net_file)
            if result is not None:
                # Offset-based mode
                self.sumo_carla_offset = result
                print(f"SUMO-CARLA coordinate offset: dx={self.sumo_carla_offset[0]:.2f}, dy={self.sumo_carla_offset[1]:.2f}")
            else:
                print("Using projection-based coordinate transformation")

    @staticmethod
    def _decode_link_state(response):
        """Return a Python state dict from either in-process or gRPC response."""
        state = getattr(response, "state", None)
        if state is not None:
            return state
        from .direct_link import parse_state_json

        return parse_state_json(getattr(response, "state_json", ""))

    @staticmethod
    def _get_net_file_from_config(config_path):
        """Extract SUMO net file path from the TeraSim scenario YAML config."""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            # Try input.sumo_net_file first, then environment.parameters.sumo_net_file_path
            net_file = config.get('input', {}).get('sumo_net_file')
            if not net_file:
                net_file = config.get('environment', {}).get('parameters', {}).get('sumo_net_file_path')
            return net_file
        except Exception as e:
            print(f"Warning: Could not read config for net file path: {e}")
            return None

    @staticmethod
    def _parse_xodr_origin(xodr_proj):
        """Extract lat_0/lon_0 and UTM zone from an xodr geoReference proj string.
        Returns (origin_lat, origin_lon, utm_zone) or (None, None, None) if not parseable.
        """
        import re
        lat_0 = lon_0 = utm_zone = None
        m = re.search(r'\+lat_0=([0-9.eE+-]+)', xodr_proj)
        if m:
            lat_0 = float(m.group(1))
        m = re.search(r'\+lon_0=([0-9.eE+-]+)', xodr_proj)
        if m:
            lon_0 = float(m.group(1))
        m = re.search(r'\+zone=(\d+)', xodr_proj)
        if m:
            utm_zone = int(m.group(1))
        return lat_0, lon_0, utm_zone

    def _calibrate_sumo_carla_offset(self, net_file):
        """Build a coordinate transformer between SUMO net.xml and CARLA (xodr) coordinate systems.

        OpenDRIVE files from Lanelet2 pipelines use coordinates that are local offsets from the
        geoReference origin (lat_0, lon_0) projected into standard UTM. pyproj ignores lat_0/lon_0
        for +proj=utm, so we handle this by:
        1. Detecting the SUMO coordinate system (EPSG:3857 for Lanelet2 conversions)
        2. Converting SUMO CRS -> standard UTM (matching xodr zone)
        3. Subtracting the geoReference origin (projected to the same UTM) to get xodr-local coords

        Returns [offset_x, offset_y] for simple offset mode, or sets self._coord_transformer
        for full projection-based conversion (returns None).
        """
        # Parse SUMO net.xml <location> for projection info
        sumo_proj = None
        sumo_net_offset = [0.0, 0.0]
        try:
            tree = ET.parse(net_file)
            root = tree.getroot()
            loc_elem = root.find('.//location')
            if loc_elem is not None:
                sumo_proj = loc_elem.get('projParameter', '!')
                offset_str = loc_elem.get('netOffset', '0.00,0.00')
                parts = offset_str.split(',')
                sumo_net_offset = [float(parts[0]), float(parts[1])]
                print(f"SUMO net.xml: projParameter='{sumo_proj}', netOffset={sumo_net_offset}")
        except Exception as e:
            print(f"Warning: Could not parse SUMO net file {net_file}: {e}")

        # Get xodr geoReference from CARLA map
        xodr_proj = None
        try:
            opendrive_str = self.world.get_map().to_opendrive()
            xodr_tree = ET.fromstring(opendrive_str)
            geo_elem = xodr_tree.find('.//geoReference')
            if geo_elem is not None and geo_elem.text:
                xodr_proj = geo_elem.text.strip()
                print(f"CARLA xodr geoReference: '{xodr_proj}'")
        except Exception as e:
            print(f"Warning: Could not get xodr geoReference from CARLA: {e}")

        # Attempt projection-based transformation with origin offset handling
        if xodr_proj:
            try:
                import pyproj

                # Parse xodr origin (lat_0, lon_0) and UTM zone
                origin_lat, origin_lon, utm_zone = self._parse_xodr_origin(xodr_proj)

                # Determine SUMO CRS
                sumo_crs = None
                if sumo_proj and sumo_proj != '!':
                    sumo_crs = pyproj.CRS(sumo_proj)
                elif sumo_proj == '!':
                    # Detect CRS empirically from coordinate ranges
                    tree = ET.parse(net_file)
                    root = tree.getroot()
                    conv_boundary = root.find('.//location').get('convBoundary', '')
                    cb_parts = conv_boundary.split(',')
                    sample_x = (float(cb_parts[0]) + float(cb_parts[2])) / 2
                    sample_y = (float(cb_parts[1]) + float(cb_parts[3])) / 2

                    wgs84 = pyproj.CRS('EPSG:4326')
                    for crs_code in ['EPSG:3857', 'EPSG:32654', 'EPSG:6677']:
                        try:
                            candidate = pyproj.CRS(crs_code)
                            to_wgs84 = pyproj.Transformer.from_crs(candidate, wgs84, always_xy=True)
                            lon, lat = to_wgs84.transform(sample_x, sample_y)
                            if 100.0 < lon < 180.0 and -60.0 < lat < 85.0:
                                sumo_crs = candidate
                                print(f"Detected SUMO CRS as {crs_code} (sample -> lon={lon:.4f}, lat={lat:.4f})")
                                break
                        except Exception:
                            continue

                if sumo_crs is None:
                    print("Warning: Could not determine SUMO CRS. Falling back to empirical calibration.")
                    return self._empirical_calibration(net_file)

                # Build transformer: SUMO CRS -> standard UTM (same zone as xodr)
                if utm_zone:
                    utm_crs = pyproj.CRS(f'EPSG:326{utm_zone:02d}')
                else:
                    # Default to UTM zone from xodr proj string
                    utm_crs = pyproj.CRS(xodr_proj)

                self._coord_transformer = pyproj.Transformer.from_crs(sumo_crs, utm_crs, always_xy=True)
                self._sumo_net_offset = sumo_net_offset

                # Compute xodr origin in standard UTM
                self._xodr_origin_utm = [0.0, 0.0]
                if origin_lat is not None and origin_lon is not None:
                    wgs84 = pyproj.CRS('EPSG:4326')
                    to_utm = pyproj.Transformer.from_crs(wgs84, utm_crs, always_xy=True)
                    ox, oy = to_utm.transform(origin_lon, origin_lat)
                    self._xodr_origin_utm = [ox, oy]
                    print(f"xodr origin ({origin_lat:.6f}, {origin_lon:.6f}) in UTM: ({ox:.2f}, {oy:.2f})")

                print(f"Using projection-based transform: SUMO -> UTM{utm_zone} - origin")
                return None  # Signal to use transformer instead of offset

            except Exception as e:
                print(f"Warning: Projection-based calibration failed: {e}")
                import traceback
                traceback.print_exc()

        # Fallback: empirical median-based calibration
        return self._empirical_calibration(net_file)

    def _empirical_calibration(self, net_file):
        """Fallback: compute offset by comparing matching road coordinates."""
        sumo_edges = {}
        try:
            tree = ET.parse(net_file)
            root = tree.getroot()
            for edge_elem in root.iter('edge'):
                edge_id = edge_elem.get('id', '')
                if edge_id.startswith(':'):
                    continue
                for lane_elem in edge_elem.iter('lane'):
                    shape_str = lane_elem.get('shape', '')
                    if shape_str:
                        points = [tuple(map(float, p.split(','))) for p in shape_str.split()]
                        mid = points[len(points) // 2]
                        sumo_edges[edge_id] = (mid[0], mid[1])
                        break
        except Exception as e:
            print(f"Warning: Could not parse net file: {e}")
            return [0.0, 0.0]

        carla_roads = {}
        try:
            for w in self.world.get_map().generate_waypoints(200.0):
                rid = str(w.road_id)
                if rid not in carla_roads:
                    carla_roads[rid] = (w.transform.location.x, w.transform.location.y)
        except Exception as e:
            print(f"Warning: Could not get CARLA waypoints: {e}")
            return [0.0, 0.0]

        dxs, dys = [], []
        for edge_id, (sx, sy) in sumo_edges.items():
            if edge_id in carla_roads:
                cx, cy = carla_roads[edge_id]
                dxs.append(cx - sx)
                dys.append(cy + sy)

        if len(dxs) < 10:
            print(f"Warning: Only {len(dxs)} matching roads. Offset may be inaccurate.")
            if not dxs:
                return [0.0, 0.0]

        offset_x = statistics.median(dxs)
        offset_y = statistics.median(dys)
        print(f"Empirical calibration from {len(dxs)} matching roads")
        return [offset_x, offset_y]

    def _wait_for_terasim_step(self):
        while True:
            response = get_terasim_status(
                self.args.terasim_host,
                self.args.terasim_port,
                self.terasim["simulation_id"],
            )
            status = response.get("status")
            if status is None:
                print("TeraSim status is None. Exiting...")
                return None
            if status in {"finished", "error"}:
                return status

            completed_tick_count = response.get("completed_tick_count")
            if completed_tick_count is not None:
                try:
                    completed_tick_count = int(completed_tick_count)
                except (TypeError, ValueError):
                    completed_tick_count = None

            if status == "wait_for_tick" and self._initial_terasim_state_pending:
                self._initial_terasim_state_pending = False
                if completed_tick_count is not None:
                    self._last_completed_terasim_tick_count = completed_tick_count
                return status

            if (
                status == "ticked"
                and completed_tick_count is not None
                and (
                    self._last_completed_terasim_tick_count is None
                    or completed_tick_count > self._last_completed_terasim_tick_count
                )
            ):
                self._initial_terasim_state_pending = False
                self._last_completed_terasim_tick_count = completed_tick_count
                return status
            time.sleep(0.05)

    def _start_step_profile(self):
        self._profile_step_count += 1
        if not self.profile_steps_enabled:
            self._current_step_profile = None
            return
        self._current_step_profile = {
            "step": self._profile_step_count,
            "simulation_time": None,
            "total_s": 0.0,
            "terasim_roundtrip": {},
            "carla_state_apply": {},
            "carla_tick": {},
            "feedback": {},
            "counts": {},
        }

    def _profile_add(self, path, elapsed_s):
        if getattr(self, "_current_step_profile", None) is None:
            return
        node = self._current_step_profile
        parts = path.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        leaf = parts[-1]
        node[leaf] = float(node.get(leaf, 0.0)) + float(elapsed_s)

    def _profile_set(self, path, value):
        if getattr(self, "_current_step_profile", None) is None:
            return
        node = self._current_step_profile
        parts = path.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value

    @contextmanager
    def _profile_timer(self, path):
        if getattr(self, "_current_step_profile", None) is None:
            yield
            return
        started_at = time.perf_counter()
        try:
            yield
        finally:
            self._profile_add(path, time.perf_counter() - started_at)

    def _complete_pending_terasim_roundtrip(self, completed_at):
        pending = self._pending_terasim_roundtrip
        if not isinstance(pending, dict):
            return
        self._profile_set(
            "terasim_roundtrip.total_s",
            max(0.0, completed_at - pending["started_at"]),
        )
        self._profile_set(
            "terasim_roundtrip.tick_request_s", pending.get("tick_request_s", 0.0)
        )
        self._pending_terasim_roundtrip = None

    def _finish_step_profile(self, total_s):
        profile = self._current_step_profile
        if profile is None:
            return
        profile["total_s"] = float(total_s)
        if isinstance(self.terasim_states, dict):
            profile["simulation_time"] = self.terasim_states.get("simulation_time")
            agent_count = self.terasim_states.get("agent_count", {})
            if isinstance(agent_count, dict):
                profile["counts"]["exported_sumo_vehicles"] = agent_count.get("vehicle", 0)
        profile["counts"]["physics_vehicles"] = len(self._physics_active_vehicle_ids)
        if self._profile_step_count <= self.profile_warmup_steps or not self.profile_jsonl_path:
            return
        profile_dir = os.path.dirname(self.profile_jsonl_path)
        if profile_dir:
            os.makedirs(profile_dir, exist_ok=True)
        with open(self.profile_jsonl_path, "a", encoding="utf-8") as profile_file:
            profile_file.write(json.dumps(profile, separators=(",", ":")) + "\n")

    def _tick_ackermann_feedback_apply_http(self):
        status = self._wait_for_terasim_step()
        if status is None or status in {"finished", "error"}:
            self._apply_ackermann_fail_closed_brake(f"terasim_status:{status}")
            return False

        self.sync_cosim_actor_to_carla()
        if not getattr(self.args, "skip_tls", False):
            self.sync_cosim_tls_to_carla()
        self.world.tick()
        self.sync_carla_ackermann_feedback_to_cosim()
        tick_terasim(
            self.args.terasim_host,
            self.args.terasim_port,
            self.terasim["simulation_id"],
        )
        return True

    def _tick_ackermann_feedback_apply_direct(self):
        if self._direct_tick_future is not None:
            wait_started_at = time.perf_counter()
            try:
                response = self._direct_tick_future.result(timeout=300.0)
            except Exception as exc:
                print(f"TeraSim link tick failed: {exc}. Exiting...")
                self._apply_ackermann_fail_closed_brake("cosim_link_tick_error")
                return False
            completion_observed_at = time.perf_counter()
            self._profile_set(
                "terasim_roundtrip.completion_wait_s",
                completion_observed_at - wait_started_at,
            )
            self._complete_pending_terasim_roundtrip(completion_observed_at)
            if response.status in ("finished", "error"):
                print(f"TeraSim ended (status={response.status}). Exiting...")
                self._apply_ackermann_fail_closed_brake(f"direct_status:{response.status}")
                return False
            with self._profile_timer("terasim_roundtrip.state_decode_s"):
                state = self._decode_link_state(response)
            if state is not None:
                self._direct_prev_state = state

        state_apply_started_at = time.perf_counter()
        if self._direct_prev_state is not None:
            self.sync_cosim_actor_to_carla(self._direct_prev_state)
            if not getattr(self.args, "skip_tls", False):
                with self._profile_timer("carla_state_apply.tls_sync_s"):
                    self.sync_cosim_tls_to_carla(self._direct_prev_state)
        self._profile_set(
            "carla_state_apply.total_s", time.perf_counter() - state_apply_started_at
        )

        carla_tick_started_at = time.perf_counter()
        with self._profile_timer("carla_tick.world_tick_s"):
            self.world.tick()
        if getattr(self, "_current_step_profile", None) is not None:
            with self._profile_timer("carla_tick.snapshot_actor_refresh_s"):
                snapshot = self.world.get_snapshot()
                self._profile_set("counts.carla_frame", snapshot.frame)
        self._profile_set("carla_tick.total_s", time.perf_counter() - carla_tick_started_at)

        with self._profile_timer("feedback.command_conversion_s"):
            commands, feedback_records = self._collect_ackermann_feedback()
        request_started_at = time.perf_counter()
        try:
            self._direct_tick_future = self.direct_link.tick_async(commands)
        except Exception as exc:
            self._finalize_ackermann_feedback_records(
                feedback_records,
                accepted=False,
                reason=f"feedback_transport_error:{type(exc).__name__}",
            )
            return False
        tick_request_s = time.perf_counter() - request_started_at
        self._pending_terasim_roundtrip = {
            "started_at": request_started_at,
            "tick_request_s": tick_request_s,
        }
        self._profile_set("feedback.command_count", len(commands))
        with self._profile_timer("feedback.bookkeeping_s"):
            transport_name = getattr(self.direct_link, "transport_name", "grpc")
            self._finalize_ackermann_feedback_records(
                feedback_records,
                accepted=True,
                reason=f"accepted_by_{transport_name}_tick",
            )
        return True

    def tick(self):
        self._start_step_profile()
        step_started_at = time.perf_counter()
        try:
            return self._tick_unprofiled()
        finally:
            self._finish_step_profile(time.perf_counter() - step_started_at)
            self._current_step_profile = None

    def _tick_unprofiled(self):
        if self.direct_link is not None:
            return self._tick_direct()
        if self.ackermann_feedback_apply_enabled:
            return self._tick_ackermann_feedback_apply_http()
        if self.async_mode:
            time_start = time.time()
            if self.control_av:
                self.sync_carla_av_to_cosim()

            self.sync_cosim_actor_to_carla()
            self.sync_cosim_tls_to_carla()

            self.world.tick()
            if self.ackermann_feedback_shadow_enabled:
                self.sync_carla_ackermann_feedback_to_cosim()
            time_end = time.time()
            elapsed = time_end - time_start
            if elapsed < self.step_length:
                time.sleep(self.step_length - elapsed)
        else:
            while True:
                terasim_status_http_response = get_terasim_status(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])
                terasim_status = terasim_status_http_response.get("status", None)
                if terasim_status == "ticked" or terasim_status == "wait_for_tick":
                    break
                elif terasim_status is None:
                    print("TeraSim status is None. Exiting...")
                    return False
                else:
                    time.sleep(0.05)

            if self.control_av:
                self.sync_carla_av_to_cosim()

            self.sync_cosim_actor_to_carla()
            if not getattr(self.args, "skip_tls", False):
                self.sync_cosim_tls_to_carla()

            tick_terasim(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])

            # 3-cosim passive mode: the psim bridge (autoware_carla_interface) is the sole
            # owner of world.tick(). CarlaCosim does not tick the world; it waits for the
            # psim tick so the two clients stay synchronized on one CARLA server.
            if getattr(self.args, "passive_tick", False):
                self.world.wait_for_tick()
            else:
                self.world.tick()
            if self.ackermann_feedback_shadow_enabled:
                self.sync_carla_ackermann_feedback_to_cosim()
        return True

    def _tick_direct(self):
        """One co-sim step over the direct gRPC link (no Redis/HTTP/polling).

        Pipeline parity with the polling path: the state rendered into CARLA is
        the previous step's state, and the SUMO step requested this tick
        computes in the background while this client waits for the CARLA tick.
        """
        if self.ackermann_feedback_apply_enabled:
            return self._tick_ackermann_feedback_apply_direct()

        # Resolve the step requested on the previous tick.
        if self._direct_tick_future is not None:
            try:
                resp = self._direct_tick_future.result(timeout=300.0)
            except Exception as e:
                print(f"TeraSim link tick failed: {e}. Exiting...")
                return False
            if resp.status in ("finished", "error"):
                print(f"TeraSim ended (status={resp.status}). Exiting...")
                return False
            state = self._decode_link_state(resp)
            if state is not None:
                self._direct_prev_state = state

        commands = []
        if self.control_av:
            av_command = self._build_av_command()
            if av_command is not None:
                commands.append(av_command)

        # Request the next SUMO step; it runs while CARLA ticks below.
        self._direct_tick_future = self.direct_link.tick_async(commands)

        # Render the previous step's state (same one-step latency as the
        # polling path, which renders the state before requesting its tick).
        if self._direct_prev_state is not None:
            self.sync_cosim_actor_to_carla(self._direct_prev_state)
            if not getattr(self.args, "skip_tls", False):
                self.sync_cosim_tls_to_carla(self._direct_prev_state)

        if getattr(self.args, "passive_tick", False):
            self.world.wait_for_tick()
        else:
            self.world.tick()
        if self.ackermann_feedback_shadow_enabled:
            self.sync_carla_ackermann_feedback_to_cosim()
        return True

    def sync_carla_av_to_cosim(self):
        """Build the AV set_state command and send it over the HTTP service link."""
        av_command = self._build_av_command()
        if av_command is None:
            return
        control_agent(
            self.args.terasim_host,
            self.args.terasim_port,
            self.terasim["simulation_id"],
            av_command,
        )

    def _build_av_command(self):
        # 3-cosim: the ego that drives in CARLA is the Autoware ego (role "ego_vehicle"), not the
        # SUMO-spawned "AV". Read that actor's pose and build a set_state command for the SUMO AV
        # so background traffic avoids it. Returns None when the command cannot be built yet
        # (actor missing / av_shape not initialized). Sending is up to the caller: the polling
        # path POSTs it (sync_carla_av_to_cosim), the direct path attaches it to the Tick RPC.
        av_role = getattr(self.args, "av_carla_role", AV_SUMO_ID)
        vehicle_status, carla_id = get_actor_id_from_attribute(self.world, av_role)

        if not vehicle_status:
            print(f"AV source actor (role={av_role}) not found in Carla simulation.")
            return None

        if not self.av_shape:
            # av_shape is filled by initialize_av in sync_cosim_actor_to_carla, which runs later in
            # the same tick. Skip until then to avoid indexing an empty shape on the first tick.
            return None

        AV = self.world.get_actor(carla_id)
        if AV is None:
            print(f"AV actor {carla_id} not resolvable this loop; command skipped.", flush=True)
            return None
        transform = AV.get_transform()
        draw_text(self.world, transform.location + carla.Location(z=2.5), AV_SUMO_ID)
        # draw_point(
        #     self.world,
        #     size=0.05,
        #     color=(255, 0, 0),
        #     location=transform.location + carla.Location(z=2.5),
        #     life_time=0,
        # )

        velocity = AV.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5

        # Reverse transform: CARLA -> SUMO
        if getattr(self, "_coord_transformer", None) is not None:
            # Direct reverse: CARLA location -> xodr coords -> UTM -> SUMO CRS
            # CARLA: x = xodr_x, y = -xodr_y
            xodr_x = transform.location.x
            xodr_y = -transform.location.y
            sumo_x, sumo_y = self._transform_xodr_to_sumo(xodr_x, xodr_y)
            # Apply vehicle shape correction (SUMO position is front bumper)
            yaw = math.radians(90.0 - (-1 * transform.rotation.yaw + 90))
            sumo_x += math.cos(yaw) * self.av_shape[0] / 2.0
            sumo_y += math.sin(yaw) * self.av_shape[0] / 2.0
            av_sumo_location = [sumo_x, sumo_y, transform.location.z]
            av_sumo_rotation = [transform.rotation.pitch, transform.rotation.yaw + 90, transform.rotation.roll]
        else:
            av_offset = [self.sumo_carla_offset[0], self.sumo_carla_offset[1], 0.0]
            av_sumo_location, av_sumo_rotation = carla_to_sumo(
                transform.location,
                transform.rotation,
                self.av_shape,
                av_offset
            )

        av_command = {
            "agent_id": AV_SUMO_ID,
            "agent_type": "vehicle",
            "command_type": "set_state",
            "data": {
                "position": [av_sumo_location[0], av_sumo_location[1]],
                "speed": speed,
                "sumo_angle": av_sumo_rotation[1],
            }
        }

        return av_command

    def _get_ackermann_feedback_shape(self, actor_id):
        vehicle_states = (
            self.terasim_states.get("agent_details", {}).get("vehicle", {})
            if isinstance(self.terasim_states, dict)
            else {}
        )
        vehicle_state = vehicle_states.get(actor_id)
        if not isinstance(vehicle_state, dict):
            return None
        shape = [
            self._as_finite_float(vehicle_state.get("length")),
            self._as_finite_float(vehicle_state.get("width")),
            self._as_finite_float(vehicle_state.get("height")),
        ]
        if any(value is None or value <= 0.0 for value in shape):
            return None
        return shape

    def _carla_transform_to_sumo_feedback_state(
        self,
        transform,
        shape,
        front_bumper_local_x=None,
        rear_axle_local_x=None,
    ):
        location = transform.location
        rotation = transform.rotation
        values = [
            self._as_finite_float(location.x),
            self._as_finite_float(location.y),
            self._as_finite_float(location.z),
            self._as_finite_float(rotation.yaw),
        ]
        if any(value is None for value in values):
            return None

        _, _, carla_z, carla_yaw = values
        if front_bumper_local_x is None:
            front_bumper_local_x = shape[0] / 2.0
        if rear_axle_local_x is None:
            rear_axle_local_x = -0.5 * getattr(
                getattr(self, "ackermann_tuning", AckermannTuning()),
                "wheel_base",
                2.8,
            )

        front_point = self._transform_local_x_point(
            transform, front_bumper_local_x
        )
        rear_axle_point = self._transform_local_x_point(
            transform, rear_axle_local_x
        )
        sumo_location = self._carla_point_to_sumo_position(*front_point)
        rear_axle_location = self._carla_point_to_sumo_position(*rear_axle_point)
        sumo_rotation = [rotation.pitch, carla_yaw + 90.0, rotation.roll]

        sumo_values = [
            self._as_finite_float(sumo_location[0]),
            self._as_finite_float(sumo_location[1]),
            self._as_finite_float(sumo_location[2]),
            self._as_finite_float(sumo_rotation[1]),
            self._as_finite_float(rear_axle_location[0]),
            self._as_finite_float(rear_axle_location[1]),
        ]
        if any(value is None for value in sumo_values):
            return None
        return {
            "position": [sumo_values[0], sumo_values[1]],
            # moveToXY only accepts x/y, so carry the physical front elevation
            # separately for route-candidate validation in the SUMO plugin.
            "position_z": sumo_values[2],
            "sumo_angle": sumo_values[3] % 360.0,
            "rear_axle_position": [sumo_values[4], sumo_values[5]],
        }

    def _is_ackermann_feedback_selected_actor(self, actor_id):
        return actor_id in self.ackermann_feedback_actor_ids or (
            actor_id != AV_SUMO_ID and self.ackermann_feedback_all_background_actors
        )

    def _new_ackermann_feedback_record(self, actor_id):
        return {
            "simulation_time": self.terasim_states.get("simulation_time", ""),
            "actor_id": actor_id,
            "feedback_mode": self.ackermann_feedback_mode,
            "feedback_status": "rejected",
        }

    def _prepare_ackermann_feedback(self, actor_id, actor, snapshot):
        feedback = self._new_ackermann_feedback_record(actor_id)
        shape = self._get_ackermann_feedback_shape(actor_id)
        if shape is None:
            feedback["feedback_reason"] = "sumo_shape_missing_or_invalid"
            return None, feedback
        if actor is None:
            feedback["feedback_reason"] = "carla_actor_missing"
            return None, feedback

        if not hasattr(self, "_ackermann_actor_state"):
            self._ackermann_actor_state = {}
        actor_state = self._ackermann_actor_state.setdefault(actor_id, {})
        if actor_state.get("physics_initialization_pending"):
            feedback["feedback_reason"] = "carla_spawn_transform_pending"
            return None, feedback

        try:
            transform = actor.get_transform()
            velocity = actor.get_velocity()
        except Exception as exc:
            feedback["feedback_reason"] = f"carla_state_error:{type(exc).__name__}"
            return None, feedback

        carla_frame = int(snapshot.frame)
        previous_frame = self._ackermann_feedback_state.get(actor_id, {}).get("source_carla_frame")
        if previous_frame is not None and carla_frame <= previous_frame:
            feedback["source_carla_frame"] = carla_frame
            feedback["feedback_reason"] = "stale_or_duplicate_carla_frame"
            return None, feedback

        self._initialize_ackermann_actor_geometry(actor, actor_id, actor_state)
        fallback_tuning = getattr(self, "ackermann_tuning", AckermannTuning())
        sumo_state = self._carla_transform_to_sumo_feedback_state(
            transform,
            shape,
            front_bumper_local_x=actor_state.get(
                "front_bumper_local_x_m", shape[0] / 2.0
            ),
            rear_axle_local_x=actor_state.get(
                "rear_axle_local_x_m",
                -0.5 * fallback_tuning.wheel_base,
            ),
        )
        speed = self._as_finite_float(horizontal_speed(velocity))
        if sumo_state is None or speed is None:
            feedback["source_carla_frame"] = carla_frame
            feedback["feedback_reason"] = "non_finite_feedback_state"
            return None, feedback

        command = {
            "agent_id": actor_id,
            "agent_type": "vehicle",
            "command_type": "set_state",
            "data": {
                "position": sumo_state["position"],
                "position_z": sumo_state["position_z"],
                "speed": speed,
                "sumo_angle": sumo_state["sumo_angle"],
                "rear_axle_position": sumo_state["rear_axle_position"],
                "source_carla_frame": carla_frame,
            },
        }
        feedback.update(
            {
                "source_carla_frame": carla_frame,
                "carla_speed": speed,
                "feedback_sumo_x": sumo_state["position"][0],
                "feedback_sumo_y": sumo_state["position"][1],
                "feedback_sumo_z": sumo_state["position_z"],
                "feedback_rear_axle_sumo_x": sumo_state["rear_axle_position"][0],
                "feedback_rear_axle_sumo_y": sumo_state["rear_axle_position"][1],
                "feedback_sumo_angle": sumo_state["sumo_angle"],
            }
        )
        return command, feedback

    def _record_ackermann_feedback(self, feedback):
        self._ackermann_feedback_state[feedback["actor_id"]] = feedback
        if getattr(self, "ackermann_feedback_log_records", True):
            print("AckermannFeedback " + json.dumps(feedback, sort_keys=True), flush=True)

    def _collect_ackermann_feedback(self):
        candidate_actor_ids = set(self._ackermann_feedback_candidate_actor_ids)
        if not candidate_actor_ids:
            vehicle_states = self.terasim_states.get("agent_details", {}).get("vehicle", {})
            candidate_actor_ids = {
                actor_id
                for actor_id in vehicle_states
                if self._is_ackermann_feedback_selected_actor(actor_id)
            }
        if not candidate_actor_ids:
            return [], []

        try:
            snapshot = self.world.get_snapshot()
        except Exception as exc:
            records = []
            for actor_id in sorted(candidate_actor_ids):
                feedback = self._new_ackermann_feedback_record(actor_id)
                feedback["feedback_reason"] = f"carla_snapshot_error:{type(exc).__name__}"
                records.append(feedback)
            return [], records

        commands = []
        records = []
        for actor_id in sorted(candidate_actor_ids):
            actor = self._ackermann_feedback_actor_index.get(actor_id)
            if actor is None:
                found, carla_id = get_actor_id_from_attribute(self.world, actor_id)
                actor = self.world.get_actor(carla_id) if found else None
            command, feedback = self._prepare_ackermann_feedback(
                actor_id, actor, snapshot
            )
            records.append(feedback)
            if command is not None:
                commands.append(command)
        return commands, records

    def _finalize_ackermann_feedback_records(
        self, records, *, accepted, reason, accepted_status="queued"
    ):
        for feedback in records:
            if "source_carla_frame" in feedback and "feedback_reason" not in feedback:
                feedback["feedback_status"] = accepted_status if accepted else "rejected"
                feedback["feedback_reason"] = reason
            self._record_ackermann_feedback(feedback)
        return bool(records) and all(
            feedback["feedback_status"] in {"queued", "shadow"} for feedback in records
        )

    def sync_carla_ackermann_feedback_to_cosim(self):
        commands, records = self._collect_ackermann_feedback()
        if self.ackermann_feedback_shadow_enabled:
            return self._finalize_ackermann_feedback_records(
                records,
                accepted=True,
                reason="not_applied",
                accepted_status="shadow",
            )

        accepted = False
        reason = "batch_command_not_queued"
        if commands:
            try:
                response = control_agents_batch(
                    self.args.terasim_host,
                    self.args.terasim_port,
                    self.terasim["simulation_id"],
                    commands,
                )
                queued_count = response.get("queued_count") if isinstance(response, dict) else None
                accepted = queued_count == len(commands)
                reason = "accepted_by_http_batch_queue" if accepted else reason
            except Exception as exc:
                reason = f"feedback_transport_error:{type(exc).__name__}"
        return self._finalize_ackermann_feedback_records(
            records, accepted=accepted, reason=reason
        )

    def sync_cosim_tls_to_carla(self, terasim_states=None):
        if terasim_states is None:
            terasim_states = get_terasim_states(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])

        if not terasim_states:
            print("terasim_states not available.")
            return
        
        if "traffic_light_details" not in terasim_states:
            print("No traffic light details available.")
            return

        terasim_tls_data = terasim_states["traffic_light_details"]

        for node_id, node_info in terasim_tls_data.items():
            sumo_tls = node_info["tls"]
            sumo_information = json.loads(node_info["information"])
            parameters = None
            for program_id, program in sumo_information["programs"].items():
                try:
                    parameters = program["parameters"]                
                    break
                except KeyError:
                    print(f"KeyError: Node ({node_id}) Program ({program}) does not have 'parameters' key.")
                    continue
            if parameters is None:
                print(f"Traffic Lights within Node ({node_id}) is not synchronized with Carla.")
                continue
            
            for i in range(len(sumo_tls)):
                param_key = f"{SUMO_CARLA_TLS_LINK_PREFIX}{i}"
                carla_landmark_ids = parameters.get(param_key, "")
                if carla_landmark_ids == "":
                    continue
                carla_landmark_ids = carla_landmark_ids.split(" ")
                for landmark_id in carla_landmark_ids:
                    light_id = int(landmark_id)
                    light_actor = self.world.get_actor(light_id)
                    if not light_actor:
                        print(f"Traffic light with ID {light_id} not found in CARLA.")
                        continue

                    # Defensive guard: CARLA may return a non-TrafficLight Actor
                    # when SUMO's TLS program parameters are not mapped to a
                    # real CARLA landmark_id (e.g. netconvert --tls.guess nets
                    # like Town01). Calling set_state on such an actor raises
                    # AttributeError and aborts the whole cosim tick.
                    if not isinstance(light_actor, carla.TrafficLight):
                        continue

                    light_state = sumo_tls[i]
                    if light_state == "G" or light_state == "g":
                        light_actor.set_state(carla.TrafficLightState.Green)
                    elif light_state == "Y" or light_state == "y":
                        light_actor.set_state(carla.TrafficLightState.Yellow)
                    elif light_state == "R" or light_state == "r":
                        light_actor.set_state(carla.TrafficLightState.Red)

    @staticmethod
    def _actor_xy(actor_info):
        try:
            return float(actor_info["x"]), float(actor_info["y"])
        except (KeyError, TypeError, ValueError):
            return None

    def _filter_actor_details_by_radius(self, vehicles, vrus):
        if not self.actor_filter_enabled:
            return vehicles, vrus

        center_info = vehicles.get(self.actor_filter_center_id)
        center_xy = self._actor_xy(center_info) if center_info is not None else None
        if center_xy is None:
            if not self._actor_filter_missing_center_warned:
                print(
                    "Warning: CARLA co-sim actor radius filter center "
                    f"{self.actor_filter_center_id!r} is not available; "
                    "using unfiltered actors.",
                    flush=True,
                )
                self._actor_filter_missing_center_warned = True
            self._actor_filter_active_vehicle_ids = set(vehicles)
            return vehicles, vrus

        center_x, center_y = center_xy
        previous_active_ids = self._actor_filter_active_vehicle_ids
        enter_radius_squared = self.actor_filter_radius * self.actor_filter_radius
        exit_radius = self.actor_filter_radius + self.actor_filter_hysteresis
        exit_radius_squared = exit_radius * exit_radius

        filtered_vehicles = {}
        active_vehicle_ids = set()
        for veh_id, veh_info in vehicles.items():
            if veh_id in {self.actor_filter_center_id, AV_SUMO_ID}:
                filtered_vehicles[veh_id] = veh_info
                active_vehicle_ids.add(veh_id)
                continue
            vehicle_xy = self._actor_xy(veh_info)
            if vehicle_xy is None:
                filtered_vehicles[veh_id] = veh_info
                active_vehicle_ids.add(veh_id)
                continue
            dx = vehicle_xy[0] - center_x
            dy = vehicle_xy[1] - center_y
            radius_squared = (
                exit_radius_squared
                if veh_id in previous_active_ids
                else enter_radius_squared
            )
            if dx * dx + dy * dy <= radius_squared:
                filtered_vehicles[veh_id] = veh_info
                active_vehicle_ids.add(veh_id)

        self._actor_filter_active_vehicle_ids = active_vehicle_ids
        return filtered_vehicles, vrus

    def sync_cosim_actor_to_carla(self, terasim_states=None):
        """Update all actors in cosim to CARLA.

        terasim_states: pass the state dict directly (direct/gRPC mode); None
        fetches it from the service over HTTP (Redis-era path).
        """
        if terasim_states is None:
            terasim_states = get_terasim_states(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])
        self.terasim_states = terasim_states or {}

        if not terasim_states:
            print("terasim_states not available.")
            return
        
        if "agent_details" not in terasim_states:
            print("No agent details available.")
            return
        
        if "vehicle" not in terasim_states["agent_details"]:
            print("No vehicle details available.")
            return
        
        if "vru" not in terasim_states["agent_details"]:
            print("No VRU details available.")
            return

        cosim_id_record = set()
        current_frame = self.world.get_snapshot().frame
        actor_index_started_at = time.perf_counter()
        vehicle_actor_index, pedestrian_actor_index = self._build_actor_role_indexes()
        self._profile_add(
            "carla_state_apply.actor_index_role_lookup_s",
            time.perf_counter() - actor_index_started_at,
        )
        self._profile_set("counts.carla_vehicle_actors", len(vehicle_actor_index))
        state_filter_started_at = time.perf_counter()
        vehicles = terasim_states["agent_details"]["vehicle"]
        vrus = terasim_states["agent_details"]["vru"]
        vehicles, vrus = self._filter_actor_details_by_radius(vehicles, vrus)
        self._update_physics_active_vehicle_ids(vehicles)
        self._profile_add(
            "carla_state_apply.state_filter_physics_selection_s",
            time.perf_counter() - state_filter_started_at,
        )
        feedback_candidate_actor_ids = {
            veh_id
            for veh_id in vehicles
            if self._is_ackermann_feedback_selected_actor(veh_id)
            and self._uses_ackermann_physics(veh_id)
        }
        transform_batch = []
        ackermann_batch = []
        spawn_requests = []

        command_conversion_started_at = time.perf_counter()
        for veh_id, veh_info in vehicles.items():
            if self.control_av and veh_id == AV_SUMO_ID:
                if not self.initialize_av:
                    self.initialize_av = True
                    self.av_shape = [
                        veh_info["length"],
                        veh_info["width"],
                        veh_info["height"],
                    ]
                    print("AV is initialized based on SUMO state.")
                # In control_av / 3-cosim mode, the CARLA-side ego actor already
                # represents the AV. Do not spawn a second SUMO AV into CARLA.
                continue

            self._process_vehicle(
                veh_id,
                veh_info,
                cosim_id_record,
                carla_actor=vehicle_actor_index.get(veh_id),
                actor_index=vehicle_actor_index,
                current_frame=current_frame,
                transform_batch=transform_batch,
                ackermann_batch=ackermann_batch,
                spawn_requests=spawn_requests,
            )
        
        for vru_id, vru_info in vrus.items():
            vru_actor_index = (
                vehicle_actor_index
                if self._vru_uses_vehicle_blueprint(vru_info)
                else pedestrian_actor_index
            )
            self._process_vru(
                vru_id,
                vru_info,
                cosim_id_record,
                carla_actor=vru_actor_index.get(vru_id),
                actor_index=vru_actor_index,
                current_frame=current_frame,
                transform_batch=transform_batch,
                spawn_requests=spawn_requests,
            )

        self._profile_add(
            "carla_state_apply.command_conversion_s",
            time.perf_counter() - command_conversion_started_at,
        )
        with self._profile_timer("carla_state_apply.actor_spawn_s"):
            self._flush_actor_spawn_batch(spawn_requests, transform_batch)
        with self._profile_timer("carla_state_apply.transform_batch_s"):
            self._flush_actor_transform_batch(transform_batch)
        with self._profile_timer("carla_state_apply.ackermann_batch_apply_s"):
            self._flush_actor_ackermann_batch(ackermann_batch)
        with self._profile_timer("carla_state_apply.actor_cleanup_s"):
            self._cleanup_actors("vehicle", "vehicle.*", cosim_id_record)
            self._cleanup_actors("pedestrian", "walker.pedestrian.*", cosim_id_record)
        self._ackermann_feedback_candidate_actor_ids = feedback_candidate_actor_ids
        self._ackermann_feedback_actor_index = {
            actor_id: vehicle_actor_index[actor_id]
            for actor_id in feedback_candidate_actor_ids
            if actor_id in vehicle_actor_index
        }
        self._prune_spawn_failures(vehicles.keys(), vrus.keys())
        self._prune_ackermann_actor_state(vehicles.keys())
        self._prune_ackermann_feedback_state(feedback_candidate_actor_ids)
        self._prune_collision_sensors(vehicles.keys())

        # self.sync_cosim_tls_to_carla()

    def sync_cosim_construction_zone_to_carla(self):
        def add_interpolated_points(points, offset):
            """
            Interpolates additional points to ensure no two consecutive points
            after UTM transformation have a distance greater than the specified offset.
            """
            refined_points = []
            print("enter add_interpolated_points")
            for i in range(len(points) - 1):
                p1 = points[i]
                p2 = points[i + 1]
                # p1 = utm_to_carla(points[i][0], points[i][1])
                # p2 = utm_to_carla(points[i + 1][0], points[i + 1][1])
                refined_points.append(p1)  # Add the current transformed point

                # Calculate the 2D distance between transformed points (x, y only)
                distance = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
                if distance > offset:
                    # Add intermediate points
                    num_new_points = int(distance // offset)
                    for j in range(1, num_new_points + 1):
                        # Linear interpolation to find new points
                        new_x = p1[0] + j * (p2[0] - p1[0]) / (num_new_points + 1)
                        new_y = p1[1] + j * (p2[1] - p1[1]) / (num_new_points + 1)
                        refined_points.append((new_x, new_y))

            refined_points.append(points[-1])  # Add the last transformed point
            return refined_points

        try:
            construction_zone_info = self.redis_client.get(CONSTRUCTION_ZONE_INFO)
            if not construction_zone_info:
                print("construction_zone_info is None or empty")
                return
        except Exception as e:
            print(f"Error fetching construction zone info: {e}")
            return

        print("entering construction zone")
        if construction_zone_info:
            closed_lane_shapes = construction_zone_info.closed_lane_shapes

            for closed_lane_shape in closed_lane_shapes:
                closed_lane_shape = add_interpolated_points(closed_lane_shape, 10)
                for cone_point in closed_lane_shape:
                    construction_cone = create_construction_zone_blueprint(self.world)
                    spawn_point = carla.Transform()
                    spawn_point.location.x, spawn_point.location.y = utm_to_carla(
                        cone_point[0], cone_point[1]
                    )
                    spawn_point.location.z = get_z_offset(
                        self.world,
                        start_location=carla.Location(
                            spawn_point.location.x, spawn_point.location.y, 300
                        ),
                        end_location=carla.Location(
                            spawn_point.location.x, spawn_point.location.y, 200
                        ),
                    )
                    id = spawn_actor(
                        client=self.client,
                        blueprint=construction_cone,
                        transform=spawn_point,
                        world=self.world,
                        actor_role="construction_cone",
                    )
                    print(f"created construction cone: {id}")

    def _transform_sumo_to_xodr(self, sumo_x, sumo_y):
        """Transform SUMO coordinates to xodr/CARLA coordinate system.

        Steps: SUMO internal coords -> raw CRS coords -> standard UTM -> subtract xodr origin.
        Returns (xodr_x, xodr_y) where xodr_y still needs to be negated for CARLA.
        """
        if self._coord_transformer is not None:
            # SUMO internal coords = raw coords + netOffset
            raw_x = sumo_x - self._sumo_net_offset[0]
            raw_y = sumo_y - self._sumo_net_offset[1]
            # Transform to standard UTM
            utm_x, utm_y = self._coord_transformer.transform(raw_x, raw_y)
            # Subtract xodr origin to get xodr-local coordinates
            xodr_x = utm_x - self._xodr_origin_utm[0]
            xodr_y = utm_y - self._xodr_origin_utm[1]
            return xodr_x, xodr_y
        return None, None

    def _transform_xodr_to_sumo(self, xodr_x, xodr_y):
        """Reverse transform: xodr/CARLA coordinates -> SUMO coordinates.

        Steps: add xodr origin -> standard UTM -> SUMO CRS -> add netOffset.
        """
        if self._coord_transformer is not None:
            # xodr-local -> standard UTM
            utm_x = xodr_x + self._xodr_origin_utm[0]
            utm_y = xodr_y + self._xodr_origin_utm[1]
            # Reverse transform: UTM -> SUMO CRS
            # _coord_transformer goes SUMO CRS -> UTM, we need the inverse
            raw_x, raw_y = self._coord_transformer.transform(utm_x, utm_y, direction='INVERSE')
            # Add netOffset
            sumo_x = raw_x + self._sumo_net_offset[0]
            sumo_y = raw_y + self._sumo_net_offset[1]
            return sumo_x, sumo_y
        return None, None

    def _get_carla_offset(self, sumo_location, z_offset):
        """Get the offset for sumo_to_carla, incorporating coordinate transformation.
        If using projection-based transform, converts SUMO coords to xodr coords and
        computes the effective offset. Otherwise, returns the calibrated static offset.
        """
        if self._coord_transformer is not None:
            xodr_x, xodr_y = self._transform_sumo_to_xodr(sumo_location[0], sumo_location[1])
            # sumo_to_carla computes: carla_x = sumo_x - cos*shape/2 + offset_x
            #                          carla_y = -(sumo_y - sin*shape/2) + offset_y
            # We want: carla_x ≈ xodr_x, carla_y ≈ -xodr_y
            # So: offset_x = xodr_x - sumo_x (approximately, ignoring shape term)
            #     offset_y = -xodr_y - (-sumo_y) = sumo_y - xodr_y
            return [xodr_x - sumo_location[0], sumo_location[1] - xodr_y, z_offset]
        return [self.sumo_carla_offset[0], self.sumo_carla_offset[1], z_offset]

    def _sumo_point_to_carla_location(self, sumo_location, z_offset=0.0):
        return sumo_point_to_carla(
            sumo_location, self._get_carla_offset(sumo_location, z_offset)
        )

    def _carla_point_to_sumo_position(self, carla_x, carla_y, carla_z=0.0):
        """Convert a physical CARLA point to SUMO coordinates without shape correction."""
        if getattr(self, "_coord_transformer", None) is not None:
            sumo_x, sumo_y = self._transform_xodr_to_sumo(carla_x, -carla_y)
        else:
            sumo_x = carla_x - self.sumo_carla_offset[0]
            sumo_y = -carla_y + self.sumo_carla_offset[1]
        return [sumo_x, sumo_y, carla_z]

    @staticmethod
    def _transform_local_x_point(transform, local_x):
        yaw = math.radians(transform.rotation.yaw)
        return (
            transform.location.x + math.cos(yaw) * local_x,
            transform.location.y + math.sin(yaw) * local_x,
            transform.location.z,
        )

    @staticmethod
    def _phase_aligned_longitudinal_error(
        current_rear_axle,
        desired_rear_axle,
        current_velocity,
        desired_heading,
        step_length,
    ):
        """Compare both rear-axle positions at the SUMO target time t + dt."""
        predicted_x = current_rear_axle[0] + float(
            getattr(current_velocity, "x", 0.0)
        ) * max(0.0, step_length)
        predicted_y = current_rear_axle[1] + float(
            getattr(current_velocity, "y", 0.0)
        ) * max(0.0, step_length)
        return (
            (desired_rear_axle[0] - predicted_x) * math.cos(desired_heading)
            + (desired_rear_axle[1] - predicted_y) * math.sin(desired_heading)
        )

    @classmethod
    def _phase_aligned_front_progress_error(
        cls,
        current_transform,
        desired_transform,
        front_bumper_local_x,
        current_velocity,
        desired_heading,
        step_length,
    ):
        """Return rear-axle progress error without mixing yaw/lateral error into it.

        SUMO and CARLA physical front positions are symmetric. Translating both
        front positions by the same rigid front-to-rear-axle offset cancels in
        their signed path-progress difference, while rotating the desired offset
        with SUMO lane yaw would incorrectly turn heading error into longitudinal
        error on curves.
        """
        current_front = cls._transform_local_x_point(
            current_transform, front_bumper_local_x
        )
        desired_front = cls._transform_local_x_point(
            desired_transform, front_bumper_local_x
        )
        return cls._phase_aligned_longitudinal_error(
            current_front,
            desired_front,
            current_velocity,
            desired_heading,
            step_length,
        )

    def _sumo_front_to_carla_transform(
        self,
        sumo_location,
        sumo_rotation,
        shape,
        offset,
        front_bumper_local_x=None,
    ):
        """Map SUMO's front-center reference point to a CARLA actor origin."""
        if front_bumper_local_x is None:
            front_bumper_local_x = shape[0] / 2.0
        front_location = sumo_point_to_carla(sumo_location, offset)
        carla_yaw = sumo_rotation[1] - 90.0
        heading = math.radians(carla_yaw)
        actor_location = carla.Location(
            x=front_location.x - math.cos(heading) * front_bumper_local_x,
            y=front_location.y - math.sin(heading) * front_bumper_local_x,
            z=front_location.z,
        )
        return carla.Transform(
            actor_location,
            carla.Rotation(
                pitch=sumo_rotation[0],
                yaw=carla_yaw,
                roll=sumo_rotation[2],
            ),
        )

    def _resolve_sumo_lookahead_location(self, veh_id, veh_info, sumo_location, sumo_angle):
        if bool(veh_info.get("lookahead_position_valid", False)):
            lookahead = [
                self._as_finite_float(veh_info.get("lookahead_x")),
                self._as_finite_float(veh_info.get("lookahead_y")),
                self._as_finite_float(veh_info.get("lookahead_z")),
            ]
            if lookahead[2] is None:
                lookahead[2] = sumo_location[2]
            if lookahead[0] is not None and lookahead[1] is not None:
                return lookahead

        desired_speed = self._resolve_ackermann_desired_speed(veh_id, veh_info)
        lookahead_distance = min(15.0, max(7.0, desired_speed))
        heading = math.radians(90.0 - sumo_angle)
        return [
            sumo_location[0] + math.cos(heading) * lookahead_distance,
            sumo_location[1] + math.sin(heading) * lookahead_distance,
            sumo_location[2],
        ]

    @staticmethod
    def _set_actor_simulate_physics(actor, enabled):
        try:
            actor.set_simulate_physics(enabled)
        except Exception:
            pass

    @staticmethod
    def _ackermann_actor_footprint(actor, transform):
        """Return an oriented 2-D physical footprint for a CARLA vehicle."""
        try:
            bounding_box = actor.bounding_box
            box_location = bounding_box.location
            extent = bounding_box.extent
            actor_yaw = math.radians(float(transform.rotation.yaw))
            box_yaw = math.radians(float(getattr(bounding_box.rotation, "yaw", 0.0)))
            center_x = (
                float(transform.location.x)
                + math.cos(actor_yaw) * float(box_location.x)
                - math.sin(actor_yaw) * float(box_location.y)
            )
            center_y = (
                float(transform.location.y)
                + math.sin(actor_yaw) * float(box_location.x)
                + math.cos(actor_yaw) * float(box_location.y)
            )
            yaw = actor_yaw + box_yaw
            forward = (math.cos(yaw), math.sin(yaw))
            left = (-math.sin(yaw), math.cos(yaw))
            center_z = float(transform.location.z) + float(box_location.z)
            return {
                "center": (center_x, center_y),
                "axes": (forward, left),
                "half_extents": (float(extent.x), float(extent.y)),
                "center_z": center_z,
                "half_z": float(extent.z),
            }
        except Exception:
            return None

    @staticmethod
    def _ackermann_footprints_overlap(first, second, clearance=0.05):
        if abs(first["center_z"] - second["center_z"]) > (
            first["half_z"] + second["half_z"] + clearance
        ):
            return False

        delta = (
            second["center"][0] - first["center"][0],
            second["center"][1] - first["center"][1],
        )
        for axis in first["axes"] + second["axes"]:
            center_distance = abs(delta[0] * axis[0] + delta[1] * axis[1])
            first_radius = sum(
                half_extent
                * abs(local_axis[0] * axis[0] + local_axis[1] * axis[1])
                for half_extent, local_axis in zip(
                    first["half_extents"], first["axes"]
                )
            )
            second_radius = sum(
                half_extent
                * abs(local_axis[0] * axis[0] + local_axis[1] * axis[1])
                for half_extent, local_axis in zip(
                    second["half_extents"], second["axes"]
                )
            )
            if center_distance > first_radius + second_radius + clearance:
                return False
        return True

    def _ackermann_spawn_transform_is_clear(self, actor, veh_id, target_transform):
        footprint = self._ackermann_actor_footprint(actor, target_transform)
        if footprint is None:
            return True, None

        for other_id, other_actor in (getattr(self, "_vehicle_actor_index", None) or {}).items():
            if other_id == veh_id or other_actor is actor:
                continue
            other_state = self._ackermann_actor_state.get(other_id, {})
            if other_state.get("physics_ground_transform_reserved"):
                other_transform = other_state.get("physics_initialization_transform")
            elif other_state.get("physics_initialization_pending"):
                # Elevated, non-physical actors do not reserve road space until
                # one of them has been admitted to the physical scene.
                continue
            else:
                try:
                    other_transform = other_actor.get_transform()
                except Exception:
                    continue
            if other_transform is None:
                continue
            other_footprint = self._ackermann_actor_footprint(
                other_actor, other_transform
            )
            if other_footprint is not None and self._ackermann_footprints_overlap(
                footprint, other_footprint
            ):
                return False, other_id
        return True, None

    @staticmethod
    def _transform_with_z_offset(transform, z_offset):
        return carla.Transform(
            carla.Location(
                x=transform.location.x,
                y=transform.location.y,
                z=transform.location.z + z_offset,
            ),
            carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw,
                roll=transform.rotation.roll,
            ),
        )

    def _prepare_ackermann_actor_physics(
        self,
        actor,
        veh_id,
        initial_speed,
        ground_transform,
        elevated_transform,
    ):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        self._set_actor_simulate_physics(actor, False)
        self._zero_ackermann_actor_motion(actor)
        state["physics_enabled"] = False
        state["physics_initialization_transform"] = ground_transform
        state["physics_initialization_speed"] = initial_speed
        state["physics_initialization_pending"] = True
        attempt = int(state.get("physics_reinitialization_count", 0)) + 1
        self._record_initialization_diagnostic(
            "prepared",
            actor,
            veh_id,
            expected_transform=ground_transform,
            expected_speed=initial_speed,
            attempt=attempt,
        )
        is_clear, blocking_actor_id = self._ackermann_spawn_transform_is_clear(
            actor, veh_id, ground_transform
        )
        if not is_clear:
            reason = f"overlap={blocking_actor_id}"
            if self._register_ackermann_initialization_failure(
                actor,
                veh_id,
                initial_speed,
                ground_transform,
                reason,
            ):
                return False
            state["physics_overlap_deferred"] = True
            state["physics_overlap_blocking_actor"] = blocking_actor_id
            state["physics_ground_transform_reserved"] = False
            actor.set_transform(elevated_transform)
            self._record_initialization_diagnostic(
                "overlap_deferred",
                actor,
                veh_id,
                expected_transform=ground_transform,
                expected_speed=initial_speed,
                attempt=attempt,
                extra={"blocking_vehicle_id": blocking_actor_id},
            )
            print(
                f"CARLA physics spawn deferred for {veh_id!r}: physical footprint "
                f"overlaps {blocking_actor_id!r}.",
                flush=True,
            )
            return False

        state["physics_ground_transform_reserved"] = True
        actor.set_transform(ground_transform)
        state["physics_ground_transform_applied_frame"] = self._current_carla_frame()
        state["physics_ground_transform_wait_pending"] = True
        # Never enable physics in the same CARLA frame as set_transform. CARLA
        # needs one completed world tick to update the body, wheels, and road
        # contacts at the requested ground transform.
        return False

    @staticmethod
    def _reset_diagnostic_path(path):
        if not path:
            return
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "w", encoding="utf-8"):
            pass

    def _append_diagnostic_jsonl(self, path, payload):
        if not path:
            return
        line = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        lock = getattr(self, "_diagnostic_lock", None)
        if lock is None:
            with open(path, "a", encoding="utf-8") as output:
                output.write(line + "\n")
            return
        with lock:
            with open(path, "a", encoding="utf-8") as output:
                output.write(line + "\n")

    @staticmethod
    def _diagnostic_vector(vector):
        if vector is None:
            return None
        return {
            axis: float(getattr(vector, axis, 0.0))
            for axis in ("x", "y", "z")
        }

    @classmethod
    def _diagnostic_transform(cls, transform):
        if transform is None:
            return None
        return {
            "location": cls._diagnostic_vector(transform.location),
            "rotation": {
                axis: float(getattr(transform.rotation, axis, 0.0))
                for axis in ("pitch", "yaw", "roll")
            },
        }

    @classmethod
    def _diagnostic_actor_snapshot(cls, actor):
        snapshot = {
            "actor_id": int(getattr(actor, "id", -1)),
            "type_id": str(getattr(actor, "type_id", "")),
        }
        try:
            snapshot["role_name"] = actor.attributes.get("role_name", "")
        except Exception:
            snapshot["role_name"] = ""
        for name, getter in (
            ("transform", "get_transform"),
            ("velocity", "get_velocity"),
            ("acceleration", "get_acceleration"),
            ("angular_velocity", "get_angular_velocity"),
        ):
            try:
                value = getattr(actor, getter)()
            except Exception:
                value = None
            snapshot[name] = (
                cls._diagnostic_transform(value)
                if name == "transform"
                else cls._diagnostic_vector(value)
            )
        try:
            control = actor.get_control()
            snapshot["control"] = {
                name: getattr(control, name, None)
                for name in (
                    "throttle", "steer", "brake", "hand_brake", "reverse", "gear"
                )
            }
        except Exception:
            snapshot["control"] = None
        return snapshot

    def _record_initialization_diagnostic(
        self,
        event_type,
        actor,
        veh_id,
        expected_transform=None,
        expected_speed=None,
        reason=None,
        attempt=None,
        extra=None,
    ):
        if not getattr(self, "initialization_diagnostics_enabled", False):
            return
        frame = self._current_carla_frame()
        payload = {
            "event": event_type,
            "wall_time": time.time(),
            "carla_frame": frame,
            "vehicle_id": veh_id,
            "attempt": attempt,
            "reason": reason,
            "expected_speed": self._as_finite_float(expected_speed),
            "expected_transform": self._diagnostic_transform(expected_transform),
            "actual": self._diagnostic_actor_snapshot(actor),
        }
        if extra:
            payload["extra"] = extra
        if event_type == "failure":
            category = str(reason or "unknown").split("=", 1)[0].split(":", 1)[0]
            counts = getattr(self, "_initialization_failure_counts", None)
            if counts is not None:
                counts[category] = int(counts.get(category, 0)) + 1
        self._append_diagnostic_jsonl(self.initialization_log_path, payload)

    def _ensure_collision_sensor(self, actor, veh_id):
        if not getattr(self, "collision_sensor_enabled", False) or actor is None:
            return None
        sensors = getattr(self, "_collision_sensors", None)
        if sensors is None:
            return None
        existing = sensors.get(veh_id)
        try:
            if existing is not None and existing.is_alive:
                return existing
        except Exception:
            pass
        sensors.pop(veh_id, None)
        try:
            blueprint = self.world.get_blueprint_library().find("sensor.other.collision")
            sensor = self.world.spawn_actor(
                blueprint,
                carla.Transform(),
                attach_to=actor,
            )
            sensor.listen(
                lambda event, vehicle_id=veh_id, vehicle_actor=actor: self._on_collision_event(
                    vehicle_id, vehicle_actor, event
                )
            )
        except Exception as exc:
            print(
                f"Warning: failed to attach CARLA collision sensor to {veh_id!r}: {exc}",
                flush=True,
            )
            return None
        sensors[veh_id] = sensor
        return sensor

    def _on_collision_event(self, veh_id, actor, event):
        frame = int(getattr(event, "frame", self._current_carla_frame() or -1))
        other = getattr(event, "other_actor", None)
        actor_id = int(getattr(actor, "id", -1))
        other_id = int(getattr(other, "id", -1))
        pair_ids = tuple(sorted((actor_id, other_id)))
        pair_key = f"{pair_ids[0]}:{pair_ids[1]}"
        frame_pair = (frame, pair_ids)
        lock = getattr(self, "_diagnostic_lock", None) or threading.Lock()
        with lock:
            self._collision_raw_event_count += 1
            duplicate_frame_pair = frame_pair in self._collision_seen_frame_pairs
            if not duplicate_frame_pair:
                self._collision_seen_frame_pairs.add(frame_pair)
                self._collision_unique_frame_count += 1
            previous_frame = self._collision_last_pair_frame.get(pair_ids)
            is_new_episode = (
                not duplicate_frame_pair
                and (
                    previous_frame is None
                    or frame - previous_frame > self.collision_episode_gap_frames
                )
            )
            if not duplicate_frame_pair:
                self._collision_last_pair_frame[pair_ids] = frame
            if is_new_episode:
                self._collision_episode_count += 1
                self._collision_episode_counts_by_pair[pair_key] = (
                    int(self._collision_episode_counts_by_pair.get(pair_key, 0)) + 1
                )
            episode_count = self._collision_episode_count
        payload = {
            "event": "carla_collision",
            "wall_time": time.time(),
            "carla_frame": frame,
            "carla_timestamp": self._as_finite_float(getattr(event, "timestamp", None)),
            "sensor_vehicle_id": veh_id,
            "pair_actor_ids": list(pair_ids),
            "pair_key": pair_key,
            "duplicate_frame_pair": duplicate_frame_pair,
            "new_episode": is_new_episode,
            "episode_count": episode_count,
            "normal_impulse": self._diagnostic_vector(
                getattr(event, "normal_impulse", None)
            ),
            "vehicle": self._diagnostic_actor_snapshot(actor),
            "other_actor": self._diagnostic_actor_snapshot(other) if other is not None else None,
        }
        self._append_diagnostic_jsonl(self.collision_log_path, payload)
        if not duplicate_frame_pair:
            other_role = ""
            try:
                other_role = other.attributes.get("role_name", "")
            except Exception:
                pass
            print(
                "CARLACollision "
                f"frame={frame} vehicle={veh_id!r} other={other_role or other_id!r} "
                f"new_episode={is_new_episode} pair={pair_key}",
                flush=True,
            )

    def _remove_collision_sensor(self, veh_id):
        sensor = (getattr(self, "_collision_sensors", None) or {}).pop(veh_id, None)
        if sensor is None:
            return
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            sensor.destroy()
        except Exception:
            pass

    def _prune_collision_sensors(self, active_vehicle_ids):
        active = set(active_vehicle_ids)
        for veh_id in list(getattr(self, "_collision_sensors", {}) or {}):
            if veh_id not in active:
                self._remove_collision_sensor(veh_id)

    def _write_collision_summary(self):
        if not getattr(self, "collision_sensor_enabled", False):
            return
        summary = {
            "raw_sensor_events": int(getattr(self, "_collision_raw_event_count", 0)),
            "unique_frame_pairs": int(getattr(self, "_collision_unique_frame_count", 0)),
            "contact_episodes": int(getattr(self, "_collision_episode_count", 0)),
            "episode_gap_frames": int(getattr(self, "collision_episode_gap_frames", 10)),
            "episodes_by_pair": dict(
                sorted((getattr(self, "_collision_episode_counts_by_pair", {}) or {}).items())
            ),
            "initialization_failures_by_reason": dict(
                sorted((getattr(self, "_initialization_failure_counts", {}) or {}).items())
            ),
        }
        path = getattr(self, "collision_summary_path", "")
        if path:
            with open(path, "w", encoding="utf-8") as output:
                json.dump(summary, output, indent=2, sort_keys=True)
                output.write("\n")
        print("CARLA collision summary " + json.dumps(summary, sort_keys=True), flush=True)

    def _shutdown_collision_sensors(self):
        for veh_id in list(getattr(self, "_collision_sensors", {}) or {}):
            self._remove_collision_sensor(veh_id)
        self._write_collision_summary()

    def _current_carla_frame(self):
        try:
            return int(self.world.get_snapshot().frame)
        except Exception:
            return None

    @staticmethod
    def _zero_ackermann_actor_motion(actor):
        zero_velocity = carla.Vector3D(0.0, 0.0, 0.0)
        try:
            actor.set_target_velocity(zero_velocity)
        except Exception:
            pass
        try:
            actor.set_target_angular_velocity(zero_velocity)
        except Exception:
            pass

    @staticmethod
    def _rotation_error_degrees(actual, expected, attribute):
        actual_value = float(getattr(actual, attribute, 0.0))
        expected_value = float(getattr(expected, attribute, 0.0))
        return abs((actual_value - expected_value + 180.0) % 360.0 - 180.0)

    def _ackermann_spawn_stability_failure(
        self,
        actor,
        expected_transform,
        expected_speed,
    ):
        try:
            actual_transform = actor.get_transform()
            velocity = actor.get_velocity()
        except Exception as exc:
            return f"state_error:{type(exc).__name__}"

        z_error = abs(
            float(actual_transform.location.z) - float(expected_transform.location.z)
        )
        horizontal_velocity = horizontal_speed(velocity)
        vertical_speed = abs(float(getattr(velocity, "z", 0.0)))
        speed_error = abs(horizontal_velocity - max(0.0, float(expected_speed or 0.0)))
        pitch_error = self._rotation_error_degrees(
            actual_transform.rotation, expected_transform.rotation, "pitch"
        )
        roll_error = self._rotation_error_degrees(
            actual_transform.rotation, expected_transform.rotation, "roll"
        )

        if z_error > self.ACKERMANN_SPAWN_MAX_Z_ERROR:
            return f"z_error={z_error:.3f}m"
        if speed_error > self.ACKERMANN_SPAWN_MAX_SPEED_ERROR:
            return f"speed_error={speed_error:.3f}m/s"
        if vertical_speed > self.ACKERMANN_SPAWN_MAX_VERTICAL_SPEED:
            return f"vertical_speed={vertical_speed:.3f}m/s"
        if (
            pitch_error > self.ACKERMANN_SPAWN_MAX_TILT_ERROR
            or roll_error > self.ACKERMANN_SPAWN_MAX_TILT_ERROR
        ):
            return f"tilt_error=pitch:{pitch_error:.3f},roll:{roll_error:.3f}deg"
        return None

    def _restart_ackermann_actor_physics_initialization(
        self,
        actor,
        veh_id,
        initial_speed,
        ground_transform,
        reason,
    ):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if self._register_ackermann_initialization_failure(
            actor,
            veh_id,
            initial_speed,
            ground_transform,
            reason,
        ):
            return False
        retry_count = int(state.get("physics_reinitialization_count", 0))
        self._set_actor_simulate_physics(actor, False)
        self._zero_ackermann_actor_motion(actor)
        state["physics_enabled"] = False
        state["physics_initialization_pending"] = True
        state["physics_initialization_transform"] = ground_transform
        state["physics_initialization_speed"] = initial_speed
        state["physics_overlap_deferred"] = True
        state["physics_ground_transform_reserved"] = False
        state["physics_stabilization_pending"] = False
        state["physics_stable_ticks"] = 0
        state.pop("physics_enabled_frame", None)
        state.pop("physics_last_stability_frame", None)
        state.pop("physics_ground_transform_applied_frame", None)
        state.pop("physics_ground_transform_wait_pending", None)
        elevated_transform = self._transform_with_z_offset(
            ground_transform, self.spawn_z_clearance
        )
        try:
            actor.set_transform(elevated_transform)
        except Exception:
            pass
        print(
            f"Warning: restarting CARLA physics initialization for {veh_id!r}: "
            f"{reason} (attempt={retry_count}).",
            flush=True,
        )
        return False

    def _register_ackermann_initialization_failure(
        self,
        actor,
        veh_id,
        initial_speed,
        ground_transform,
        reason,
    ):
        """Count one failed initialization attempt and abandon at the limit."""
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        failure_count = int(state.get("physics_reinitialization_count", 0)) + 1
        state["physics_reinitialization_count"] = failure_count
        self._record_initialization_diagnostic(
            "failure",
            actor,
            veh_id,
            expected_transform=ground_transform,
            expected_speed=initial_speed,
            reason=reason,
            attempt=failure_count,
        )
        max_attempts = max(1, int(getattr(self, "spawn_max_attempts", 3)))
        if failure_count < max_attempts:
            return False

        state["physics_initialization_abandoned"] = True
        state["physics_initialization_pending"] = True
        state["physics_enabled"] = False
        self._set_actor_simulate_physics(actor, False)
        self._zero_ackermann_actor_motion(actor)
        self._record_initialization_diagnostic(
            "abandoned",
            actor,
            veh_id,
            expected_transform=ground_transform,
            expected_speed=initial_speed,
            reason=reason,
            attempt=failure_count,
        )
        self._mark_spawn_abandoned("vehicle", veh_id, failure_count, reason)
        self._remove_collision_sensor(veh_id)

        actor_index = getattr(self, "_vehicle_actor_index", None)
        if actor_index is not None and actor_index.get(veh_id) is actor:
            actor_index.pop(veh_id, None)
        pending_entries = getattr(self, "_pending_actor_index_entries", None)
        if pending_entries is not None:
            pending_entries.pop(veh_id, None)

        try:
            self.client.apply_batch_sync(
                [carla.command.DestroyActor(actor.id)],
                False,
            )
        except Exception:
            try:
                actor.destroy()
            except Exception:
                pass
        print(
            f"Warning: abandoning CARLA spawn for {veh_id!r} after "
            f"{failure_count} failed initialization attempts: {reason}. "
            "SUMO simulation will continue without this CARLA actor.",
            flush=True,
        )
        return True

    def _monitor_ackermann_actor_physics_stability(
        self,
        actor,
        veh_id,
        initial_speed,
        initial_transform,
    ):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        current_frame = self._current_carla_frame()
        enabled_frame = state.get("physics_enabled_frame")
        last_frame = state.get("physics_last_stability_frame")
        if current_frame is not None:
            if enabled_frame is not None and current_frame <= enabled_frame:
                return False
            if last_frame is not None and current_frame <= last_frame:
                return False

        failure = self._ackermann_spawn_stability_failure(
            actor,
            initial_transform,
            initial_speed,
        )
        if failure is not None:
            return self._restart_ackermann_actor_physics_initialization(
                actor,
                veh_id,
                initial_speed,
                initial_transform,
                failure,
            )

        state["physics_last_stability_frame"] = current_frame
        stable_ticks = int(state.get("physics_stable_ticks", 0)) + 1
        state["physics_stable_ticks"] = stable_ticks
        if stable_ticks < self.ACKERMANN_SPAWN_STABILITY_TICKS:
            return False

        state["physics_stabilization_pending"] = False
        state["physics_initialization_pending"] = False
        state.pop("physics_ground_transform_reserved", None)
        state.pop("physics_initialization_transform", None)
        state.pop("physics_initialization_speed", None)
        self._record_initialization_diagnostic(
            "stable",
            actor,
            veh_id,
            expected_transform=initial_transform,
            expected_speed=initial_speed,
            attempt=int(state.get("physics_reinitialization_count", 0)) + 1,
            extra={"stable_ticks": stable_ticks},
        )
        print(
            f"CARLA physics initialization stable for {veh_id!r} after "
            f"{stable_ticks} tick(s).",
            flush=True,
        )
        return True

    def _initialize_ackermann_actor_velocity(self, actor, veh_id, speed, transform):
        initial_speed = self._as_finite_float(speed)
        if initial_speed is None:
            return False
        initial_speed = max(0.0, initial_speed)
        yaw = math.radians(transform.rotation.yaw)
        velocity = carla.Vector3D(
            initial_speed * math.cos(yaw),
            initial_speed * math.sin(yaw),
            0.0,
        )
        try:
            actor.set_target_velocity(velocity)
        except Exception as exc:
            print(
                f"Warning: failed to initialize CARLA velocity for {veh_id!r}: {exc}",
                flush=True,
            )
            return False

        print(
            f"CARLA initial velocity applied to {veh_id!r}: {initial_speed:.3f}m/s.",
            flush=True,
        )
        return True

    def _initialize_ackermann_actor_geometry(self, actor, veh_id, state):
        if state.get("geometry_from_physics"):
            return
        # Immediately after spawning, CARLA 0.9.16 may temporarily report the
        # actor transform as (0, 0) while wheel positions already contain
        # world-space centimetres. Retry on subsequent feedback frames.
        geometry_attempts = int(state.get("geometry_attempts", 0))
        if geometry_attempts >= 10:
            return
        state["geometry_attempts"] = geometry_attempts + 1
        state["geometry_attempted"] = True
        fallback_tuning = getattr(self, "ackermann_tuning", AckermannTuning())
        fallback_wheel_base = float(fallback_tuning.wheel_base)
        state["wheel_base_m"] = fallback_wheel_base
        state["rear_axle_local_x_m"] = -0.5 * fallback_wheel_base

        try:
            bounding_box = actor.bounding_box
            front_bumper_local_x = float(
                bounding_box.location.x + bounding_box.extent.x
            )
            if 0.1 <= front_bumper_local_x <= 10.0:
                state["front_bumper_local_x_m"] = front_bumper_local_x
                state["front_bumper_from_bounding_box"] = True
        except Exception:
            pass

        try:
            physics_control = actor.get_physics_control()
            wheels = list(getattr(physics_control, "wheels", ()) or ())
            wheel_positions = [
                (float(wheel.position.x), float(wheel.position.y))
                for wheel in wheels
                if getattr(wheel, "position", None) is not None
            ]
        except Exception:
            return
        if len(wheel_positions) < 2:
            return

        # CARLA 0.9.16 can expose wheel positions as world coordinates in
        # centimetres, while test doubles and other releases use actor-local
        # coordinates. Validate both interpretations and prefer the geometry
        # whose axle midpoint is closest to the actor origin.
        scale = (
            0.01
            if max(abs(value) for point in wheel_positions for value in point) > 20.0
            else 1.0
        )
        scaled_positions = [(x * scale, y * scale) for x, y in wheel_positions]

        def resolve_geometry(local_x_values):
            local_x_values = sorted(local_x_values)
            split = max(1, len(local_x_values) // 2)
            rear_axle_x = sum(local_x_values[:split]) / split
            front_values = local_x_values[split:]
            if not front_values:
                return None
            front_axle_x = sum(front_values) / len(front_values)
            wheel_base = front_axle_x - rear_axle_x
            if not (0.5 <= wheel_base <= 6.0 and -5.0 <= rear_axle_x <= 2.0):
                return None
            return rear_axle_x, front_axle_x, wheel_base

        candidates = []
        direct_geometry = resolve_geometry([x for x, _ in scaled_positions])
        if direct_geometry is not None:
            candidates.append(direct_geometry)

        try:
            transform = actor.get_transform()
            yaw = math.radians(float(transform.rotation.yaw))
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            world_local_x = [
                cos_yaw * (x - float(transform.location.x))
                + sin_yaw * (y - float(transform.location.y))
                for x, y in scaled_positions
            ]
            world_geometry = resolve_geometry(world_local_x)
            if world_geometry is not None:
                candidates.append(world_geometry)
        except Exception:
            pass

        if not candidates:
            return
        rear_axle_x, front_axle_x, wheel_base = min(
            candidates,
            key=lambda geometry: abs(0.5 * (geometry[0] + geometry[1])),
        )
        state["wheel_base_m"] = wheel_base
        state["rear_axle_local_x_m"] = rear_axle_x
        state["geometry_from_physics"] = True

    def _ensure_ackermann_actor_physics(
        self,
        actor,
        veh_id,
        initial_speed=None,
        initial_transform=None,
    ):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if state.get("physics_stabilization_pending"):
            expected_transform = initial_transform or state.get(
                "physics_initialization_transform"
            )
            expected_speed = (
                initial_speed
                if initial_speed is not None
                else state.get("physics_initialization_speed", 0.0)
            )
            if expected_transform is None:
                return False
            return self._monitor_ackermann_actor_physics_stability(
                actor,
                veh_id,
                expected_speed,
                expected_transform,
            )

        if not state.get("physics_enabled"):
            if state.get("physics_overlap_deferred"):
                if initial_transform is None:
                    return False
                state["physics_initialization_transform"] = initial_transform
                state["physics_initialization_speed"] = initial_speed
                state["physics_initialization_pending"] = True
                is_clear, blocking_actor_id = self._ackermann_spawn_transform_is_clear(
                    actor, veh_id, initial_transform
                )
                if not is_clear:
                    reason = f"overlap={blocking_actor_id}"
                    if self._register_ackermann_initialization_failure(
                        actor,
                        veh_id,
                        initial_speed,
                        initial_transform,
                        reason,
                    ):
                        return False
                    state["physics_overlap_blocking_actor"] = blocking_actor_id
                    actor.set_transform(
                        self._transform_with_z_offset(
                            initial_transform, self.spawn_z_clearance
                        )
                    )
                    return False
                state["physics_overlap_deferred"] = False
                state.pop("physics_overlap_blocking_actor", None)
                state["physics_ground_transform_reserved"] = True
                actor.set_transform(initial_transform)
                state["physics_ground_transform_applied_frame"] = (
                    self._current_carla_frame()
                )
                state["physics_ground_transform_wait_pending"] = True
                return False

            pending_transform = state.get("physics_initialization_transform")
            if pending_transform is None and initial_transform is not None:
                pending_transform = initial_transform
                state["physics_initialization_transform"] = initial_transform
                state["physics_initialization_speed"] = initial_speed
                state["physics_initialization_pending"] = True

            if pending_transform is not None:
                if state.get("physics_ground_transform_wait_pending"):
                    applied_frame = state.get("physics_ground_transform_applied_frame")
                    current_frame = self._current_carla_frame()
                    if (
                        applied_frame is not None
                        and current_frame is not None
                        and current_frame <= applied_frame
                    ):
                        return False
                    if applied_frame is None or current_frame is None:
                        state["physics_ground_transform_wait_pending"] = False
                        return False
                    state["physics_ground_transform_wait_pending"] = False
                try:
                    actual_transform = actor.get_transform()
                    dx = actual_transform.location.x - pending_transform.location.x
                    dy = actual_transform.location.y - pending_transform.location.y
                    dz = actual_transform.location.z - pending_transform.location.z
                    yaw_error = self._rotation_error_degrees(
                        actual_transform.rotation, pending_transform.rotation, "yaw"
                    )
                    transform_ready = (
                        math.hypot(dx, dy) <= 1.0
                        and abs(dz) <= self.ACKERMANN_SPAWN_MAX_Z_ERROR
                        and yaw_error <= 5.0
                    )
                except Exception:
                    transform_ready = False
                    actual_transform = None
                if not transform_ready:
                    return self._restart_ackermann_actor_physics_initialization(
                        actor,
                        veh_id,
                        initial_speed,
                        pending_transform,
                        "ground_transform_not_ready",
                    )
                is_clear, blocking_actor_id = self._ackermann_spawn_transform_is_clear(
                    actor, veh_id, pending_transform
                )
                if not is_clear:
                    reason = f"overlap={blocking_actor_id}"
                    if self._register_ackermann_initialization_failure(
                        actor,
                        veh_id,
                        initial_speed,
                        pending_transform,
                        reason,
                    ):
                        return False
                    state["physics_overlap_deferred"] = True
                    state["physics_overlap_blocking_actor"] = blocking_actor_id
                    state["physics_ground_transform_reserved"] = False
                    actor.set_transform(
                        self._transform_with_z_offset(
                            pending_transform, self.spawn_z_clearance
                        )
                    )
                    return False
            else:
                actual_transform = initial_transform

            self._zero_ackermann_actor_motion(actor)
            self._set_actor_simulate_physics(actor, True)
            state["physics_enabled"] = True
            self._record_initialization_diagnostic(
                "physics_enabled",
                actor,
                veh_id,
                expected_transform=pending_transform or initial_transform,
                expected_speed=initial_speed,
                attempt=int(state.get("physics_reinitialization_count", 0)) + 1,
            )
            if initial_speed is not None and initial_transform is not None:
                state["initial_velocity_applied"] = self._initialize_ackermann_actor_velocity(
                    actor,
                    veh_id,
                    initial_speed,
                    actual_transform or initial_transform,
                )
                state["physics_initialization_speed"] = initial_speed
            if pending_transform is not None:
                state["physics_stabilization_pending"] = True
                state["physics_stable_ticks"] = 0
                state["physics_enabled_frame"] = self._current_carla_frame()
                state.pop("physics_last_stability_frame", None)
            else:
                state["physics_initialization_pending"] = False

        self._initialize_ackermann_actor_geometry(actor, veh_id, state)
        if not state.get("controller_settings_attempted"):
            state["controller_settings_attempted"] = True
            tuning = self.ackermann_controller_tuning
            try:
                settings = carla.AckermannControllerSettings(
                    tuning.speed_kp,
                    tuning.speed_ki,
                    tuning.speed_kd,
                    tuning.accel_kp,
                    tuning.accel_ki,
                    tuning.accel_kd,
                )
                actor.apply_ackermann_controller_settings(settings)
            except Exception as exc:
                state["controller_settings_applied"] = False
                print(
                    f"Warning: failed to apply CARLA Ackermann controller settings "
                    f"for {veh_id!r}: {exc}",
                    flush=True,
                )
            else:
                state["controller_settings_applied"] = True
                print(
                    f"CARLA Ackermann controller settings applied to {veh_id!r}: "
                    f"speed=({tuning.speed_kp:.4g},{tuning.speed_ki:.4g},{tuning.speed_kd:.4g}) "
                    f"accel=({tuning.accel_kp:.4g},{tuning.accel_ki:.4g},{tuning.accel_kd:.4g})",
                    flush=True,
                )

        return not state.get("physics_initialization_pending", False)

    def _ensure_actor_teleport_mode(self, actor, veh_id):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if state.get("physics_enabled") is False:
            return
        self._set_actor_simulate_physics(actor, False)
        state.clear()
        state["physics_enabled"] = False

    def _warn_ackermann_position_error(self, veh_id, position_error):
        if self.ackermann_warn_error_m <= 0.0 or position_error < self.ackermann_warn_error_m:
            return
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        now = time.monotonic()
        last_warning_time = state.get("last_warning_time", 0.0)
        if now - last_warning_time < self.ackermann_warning_interval:
            return
        state["last_warning_time"] = now
        print(
            f"Warning: Ackermann vehicle {veh_id!r} is {position_error:.2f}m "
            "from its SUMO desired position.",
            flush=True,
        )

    def _is_ackermann_feedback_apply_actor(self, veh_id):
        return self.ackermann_feedback_apply_enabled and (
            self._is_ackermann_feedback_selected_actor(veh_id)
        )

    def _update_physics_active_vehicle_ids(self, vehicles):
        if not self.physics_radius_enabled:
            return
        center_info = vehicles.get(self.physics_radius_center_id)
        center_xy = self._actor_xy(center_info) if center_info is not None else None
        if center_xy is None:
            self._physics_active_vehicle_ids = {AV_SUMO_ID}
            return

        previous_active_ids = self._physics_active_vehicle_ids
        enter_radius_sq = self.physics_radius**2
        exit_radius_sq = (self.physics_radius + self.physics_radius_hysteresis) ** 2
        active_ids = {AV_SUMO_ID, self.physics_radius_center_id}
        center_x, center_y = center_xy
        for vehicle_id, vehicle_info in vehicles.items():
            if vehicle_id in active_ids:
                continue
            vehicle_xy = self._actor_xy(vehicle_info)
            if vehicle_xy is None:
                continue
            dx = vehicle_xy[0] - center_x
            dy = vehicle_xy[1] - center_y
            radius_sq = (
                exit_radius_sq if vehicle_id in previous_active_ids else enter_radius_sq
            )
            if dx * dx + dy * dy <= radius_sq:
                active_ids.add(vehicle_id)
        self._physics_active_vehicle_ids = active_ids

    def _uses_ackermann_physics(self, veh_id):
        if not self.ackermann_physics_enabled:
            return False
        if getattr(self, "physics_radius_enabled", False):
            return veh_id in self._physics_active_vehicle_ids
        if self.ackermann_feedback_apply_enabled:
            return self._is_ackermann_feedback_apply_actor(veh_id)
        return True

    def _is_ackermann_feedback_healthy(self, veh_id, feedback, observed_frame):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if feedback.get("feedback_status") != "queued":
            state["feedback_ack_failures"] = 0
            return False

        expected_frame = self._as_finite_float(feedback.get("source_carla_frame"))
        observed_frame = self._as_finite_float(observed_frame)
        frame_lag = (
            expected_frame - observed_frame
            if expected_frame is not None and observed_frame is not None
            else None
        )
        ack_missing = frame_lag is None or frame_lag > self.ackermann_feedback_ack_max_frame_lag
        failures = state.get("feedback_ack_failures", 0) + 1 if ack_missing else 0
        state["feedback_ack_failures"] = failures
        state["feedback_frame_lag"] = frame_lag
        return failures < self.ackermann_feedback_ack_failure_limit

    def _resolve_ackermann_desired_speed(self, veh_id, veh_info):
        speed_key = (
            "sumo_desired_speed"
            if self._is_ackermann_feedback_apply_actor(veh_id)
            else "speed"
        )
        desired_speed = self._as_finite_float(veh_info.get(speed_key))
        if desired_speed is None and speed_key != "speed":
            desired_speed = self._as_finite_float(veh_info.get("speed"))
        return max(0.0, desired_speed or 0.0)

    def _resolve_ackermann_max_decel(self, veh_info):
        emergency_decel = self._as_finite_float(veh_info.get("sumo_emergency_decel"))
        if emergency_decel is not None and emergency_decel > 0.0:
            return emergency_decel
        return self.ackermann_tuning.max_decel

    def _resolve_ackermann_longitudinal_target(
        self,
        veh_id,
        veh_info,
        current_speed,
        longitudinal_error=0.0,
    ):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        max_decel = self._resolve_ackermann_max_decel(veh_info)
        state["sumo_emergency_decel"] = max_decel
        desired_speed = self._resolve_ackermann_desired_speed(veh_id, veh_info)
        requested_acceleration = self._as_finite_float(veh_info.get("acceleration"))
        state["sumo_requested_acceleration"] = requested_acceleration
        if not self._is_ackermann_feedback_apply_actor(veh_id):
            state["applied_desired_acceleration"] = None
            return desired_speed, None

        sumo_next_speed = self._as_finite_float(veh_info.get("sumo_desired_speed"))
        observed_speed = self._as_finite_float(veh_info.get("feedback_observed_speed"))
        longitudinal_error = self._as_finite_float(longitudinal_error) or 0.0
        state["sumo_desired_speed"] = sumo_next_speed
        state["feedback_observed_speed"] = observed_speed
        state["longitudinal_position_error"] = longitudinal_error
        if sumo_next_speed is None or self.step_length <= 0.0:
            state["sumo_requested_acceleration"] = None
            state["applied_desired_acceleration"] = None
            return desired_speed, None

        if requested_acceleration is None:
            acceleration_origin_speed = (
                observed_speed if observed_speed is not None else current_speed
            )
            requested_acceleration = (
                sumo_next_speed - acceleration_origin_speed
            ) / self.step_length
        velocity_error = sumo_next_speed - current_speed
        desired_acceleration = min(
            self.ackermann_tuning.max_accel,
            max(
                -max_decel,
                requested_acceleration
                + self.ackermann_tuning.kp_position * longitudinal_error
                + self.ackermann_tuning.kp_speed * velocity_error,
            ),
        )
        state["sumo_requested_acceleration"] = requested_acceleration
        state["longitudinal_velocity_error"] = velocity_error
        state["applied_desired_acceleration"] = desired_acceleration
        speed_target = max(
            0.0,
            sumo_next_speed
            + self.ackermann_tuning.position_speed_gain * longitudinal_error,
        )
        return speed_target, desired_acceleration

    def _current_direct_brake_steer(self, veh_id, vehicle):
        try:
            applied_steer = self._as_finite_float(vehicle.get_control().steer)
        except Exception:
            applied_steer = None
        if applied_steer is not None:
            return min(1.0, max(-1.0, applied_steer))

        state = self._ackermann_actor_state.setdefault(veh_id, {})
        ackermann_steer = self._as_finite_float(state.get("steer")) or 0.0
        max_steer = self.ackermann_tuning.max_steer_rad
        if max_steer <= 0.0:
            return 0.0
        return min(1.0, max(-1.0, ackermann_steer / max_steer))

    def _make_direct_brake_control(self, veh_id, vehicle, brake=1.0):
        return carla.VehicleControl(
            throttle=0.0,
            steer=self._current_direct_brake_steer(veh_id, vehicle),
            brake=min(1.0, max(0.0, float(brake))),
            hand_brake=False,
            reverse=False,
        )

    def _update_ackermann_emergency_brake(self, veh_id, vehicle, current_speed):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        tuning = getattr(
            self,
            "ackermann_emergency_brake_tuning",
            AckermannEmergencyBrakeTuning(),
        )
        requested_acceleration = self._as_finite_float(
            state.get("sumo_requested_acceleration")
        )
        active = bool(state.get("emergency_brake_active", False))
        release_ticks = int(state.get("emergency_brake_release_ticks", 0))

        if not tuning.enabled:
            active = False
            release_ticks = 0
        elif active:
            if current_speed <= tuning.stop_speed:
                active = False
                release_ticks = 0
            elif (
                requested_acceleration is not None
                and requested_acceleration >= -tuning.release_decel
            ):
                release_ticks += 1
                if release_ticks >= tuning.release_ticks:
                    active = False
                    release_ticks = 0
            else:
                release_ticks = 0
        elif (
            current_speed > tuning.stop_speed
            and requested_acceleration is not None
            and requested_acceleration <= -tuning.engage_decel
        ):
            active = True
            release_ticks = 0

        state["emergency_brake_active"] = active
        state["emergency_brake_release_ticks"] = release_ticks
        state["control_mode"] = "emergency_brake" if active else "ackermann"
        if not active:
            state["emergency_brake_command"] = 0.0
            return None

        max_decel = self._as_finite_float(state.get("sumo_emergency_decel"))
        if max_decel is None or max_decel <= 0.0:
            max_decel = self.ackermann_tuning.max_decel
        brake = compute_direct_brake_value(
            requested_acceleration or 0.0,
            max_decel,
            tuning.min_brake,
        )
        state["emergency_brake_command"] = brake
        return self._make_direct_brake_control(veh_id, vehicle, brake)

    def _neutralize_ackermann_steer(self, veh_id):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        steer = self._as_finite_float(state.get("steer")) or 0.0
        max_delta = self.ackermann_tuning.max_steer_rate_rad_s * self.step_length
        if max_delta <= 0.0 or abs(steer) <= max_delta:
            return 0.0
        return steer - math.copysign(max_delta, steer)

    def _make_brake_ackermann_control(self, veh_id):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if self._is_ackermann_feedback_apply_actor(veh_id):
            steer = self._neutralize_ackermann_steer(veh_id)
            state["steer"] = steer
        else:
            steer = self._as_finite_float(state.get("steer")) or 0.0
        max_decel = self._as_finite_float(state.get("sumo_emergency_decel"))
        if max_decel is None or max_decel <= 0.0:
            max_decel = self.ackermann_tuning.max_decel
        return carla.VehicleAckermannControl(
            steer=steer,
            speed=0.0,
            acceleration=max_decel,
            jerk=0.0,
        )

    def _apply_ackermann_fail_closed_brake(self, reason):
        actors = []
        for actor_id, actor in list(self._ackermann_feedback_actor_index.items()):
            if actor is not None and self._is_ackermann_feedback_selected_actor(actor_id):
                actors.append((actor_id, actor))
        for actor_id, actor in actors:
            try:
                actor.apply_control(
                    self._make_direct_brake_control(actor_id, actor, brake=1.0)
                )
            except Exception as exc:
                print(
                    f"Warning: fail-closed brake failed for {actor_id}: {exc}",
                    flush=True,
                )
        if actors:
            print(
                f"Ackermann fail-closed direct brake applied to "
                f"{len(actors)} actor(s): {reason}",
                flush=True,
            )
            if not getattr(self.args, "passive_tick", False):
                try:
                    self.world.tick()
                except Exception as exc:
                    print(f"Warning: fail-closed CARLA tick failed: {exc}", flush=True)
        return len(actors)

    def _build_ackermann_control(
        self,
        veh_id,
        veh_info,
        vehicle,
        sumo_location,
        sumo_angle,
        desired_transform,
    ):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        try:
            current_transform = vehicle.get_transform()
            current_velocity = vehicle.get_velocity()
        except Exception:
            state["control_mode"] = "ackermann"
            return self._make_brake_ackermann_control(veh_id)

        desired_location = desired_transform.location
        current_location = current_transform.location
        rear_axle_local_x = state.get(
            "rear_axle_local_x_m",
            -0.5 * self.ackermann_tuning.wheel_base,
        )
        current_rear_axle = self._transform_local_x_point(
            current_transform, rear_axle_local_x
        )
        desired_rear_axle = self._transform_local_x_point(
            desired_transform, rear_axle_local_x
        )
        position_error = math.hypot(
            current_rear_axle[0] - desired_rear_axle[0],
            current_rear_axle[1] - desired_rear_axle[1],
        )
        self._warn_ackermann_position_error(veh_id, position_error)

        if (
            self.ackermann_snap_error_m > 0.0
            and position_error >= self.ackermann_snap_error_m
            and not state.get("snap_used", False)
            and not self._is_ackermann_feedback_apply_actor(veh_id)
        ):
            try:
                vehicle.set_transform(desired_transform)
                current_transform = desired_transform
                current_location = desired_location
                state["steer"] = 0.0
                state["snap_used"] = True
            except Exception:
                pass

        lookahead_sumo_location = self._resolve_sumo_lookahead_location(
            veh_id, veh_info, sumo_location, sumo_angle
        )
        try:
            lookahead_location = self._sumo_point_to_carla_location(
                lookahead_sumo_location
            )
        except Exception:
            state["control_mode"] = "ackermann"
            return self._make_brake_ackermann_control(veh_id)

        current_speed = horizontal_speed(current_velocity)
        desired_heading = math.radians(desired_transform.rotation.yaw)
        longitudinal_error = self._as_finite_float(
            veh_info.get("feedback_longitudinal_error")
        )
        if longitudinal_error is None:
            longitudinal_error = self._phase_aligned_front_progress_error(
                current_transform,
                desired_transform,
                state.get(
                    "front_bumper_local_x_m",
                    0.5 * float(veh_info.get("length", 5.0)),
                ),
                current_velocity,
                desired_heading,
                self.step_length,
            )
        desired_speed, feedback_desired_acceleration = (
            self._resolve_ackermann_longitudinal_target(
                veh_id,
                veh_info,
                current_speed,
                longitudinal_error=longitudinal_error,
            )
        )
        values = compute_ackermann_control_values(
            current_x=current_location.x,
            current_y=current_location.y,
            yaw_degrees=current_transform.rotation.yaw,
            current_speed=current_speed,
            desired_x=desired_location.x,
            desired_y=desired_location.y,
            lookahead_x=lookahead_location.x,
            lookahead_y=lookahead_location.y,
            desired_speed=desired_speed,
            previous_steer=state.get("steer"),
            dt=self.step_length,
            tuning=self.ackermann_tuning,
            control_point_local_x=state.get(
                "rear_axle_local_x_m",
                -0.5 * self.ackermann_tuning.wheel_base,
            ),
            wheel_base=state.get("wheel_base_m", self.ackermann_tuning.wheel_base),
        )
        final_steer = values.steer
        final_speed = values.speed
        final_acceleration = (
            values.acceleration
            if feedback_desired_acceleration is None
            else feedback_desired_acceleration
        )
        feedback = self._ackermann_feedback_state.get(veh_id, {})
        feedback_unhealthy = (
            self._is_ackermann_feedback_apply_actor(veh_id)
            and feedback
            and not self._is_ackermann_feedback_healthy(
                veh_id, feedback, veh_info.get("feedback_source_carla_frame")
            )
        )
        target_behind = (
            self._is_ackermann_feedback_apply_actor(veh_id)
            and values.lookahead_local_x <= 0.0
        )
        if feedback_unhealthy or target_behind:
            final_steer = self._neutralize_ackermann_steer(veh_id)
            final_speed = 0.0
            final_acceleration = -self._resolve_ackermann_max_decel(veh_info)

        emergency_control = self._update_ackermann_emergency_brake(
            veh_id, vehicle, current_speed
        )
        if emergency_control is None:
            control_mode = "ackermann"
            commanded_throttle = None
            commanded_brake = None
            commanded_steer = final_steer
        else:
            control_mode = "emergency_brake"
            commanded_throttle = emergency_control.throttle
            commanded_brake = emergency_control.brake
            commanded_steer = emergency_control.steer

        self._record_ackermann_control_trace(
            veh_id=veh_id,
            veh_info=veh_info,
            vehicle=vehicle,
            current_transform=current_transform,
            current_speed=current_speed,
            target_speed=final_speed,
            target_acceleration=final_acceleration,
            position_error=position_error,
            feedback_unhealthy=bool(feedback_unhealthy),
            target_behind=bool(target_behind),
            control_values=values,
            control_mode=control_mode,
            commanded_throttle=commanded_throttle,
            commanded_brake=commanded_brake,
            commanded_steer=commanded_steer,
        )
        state["steer"] = final_steer
        state["last_position_error"] = position_error
        if emergency_control is not None:
            return emergency_control
        return carla.VehicleAckermannControl(
            steer=final_steer,
            speed=final_speed,
            # CARLA 0.9.16 takes Abs(acceleration) and uses it as the symmetric
            # acceleration/deceleration limit. Target speed determines direction.
            acceleration=abs(final_acceleration),
            jerk=values.jerk,
        )

    def _record_ackermann_control_trace(
        self,
        *,
        veh_id,
        veh_info,
        vehicle,
        current_transform,
        current_speed,
        target_speed,
        target_acceleration,
        position_error,
        feedback_unhealthy,
        target_behind,
        control_values=None,
        control_mode="ackermann",
        commanded_throttle=None,
        commanded_brake=None,
        commanded_steer=None,
    ):
        if not getattr(self, "ackermann_control_log_records", False):
            return

        control_log_actor_ids = getattr(
            self, "ackermann_control_log_actor_ids", set()
        )
        if (
            control_log_actor_ids
            and "*" not in control_log_actor_ids
            and veh_id not in control_log_actor_ids
        ):
            return

        state = self._ackermann_actor_state.setdefault(veh_id, {})
        try:
            snapshot = self.world.get_snapshot()
            carla_frame = int(snapshot.frame)
        except Exception:
            carla_frame = None
        previous_speed = self._as_finite_float(state.get("trace_previous_speed"))
        previous_frame = state.get("trace_previous_frame")
        speed_delta_acceleration = None
        if (
            previous_speed is not None
            and carla_frame is not None
            and isinstance(previous_frame, int)
            and carla_frame > previous_frame
        ):
            dt = (carla_frame - previous_frame) * self.step_length
            if dt > 0.0:
                speed_delta_acceleration = (current_speed - previous_speed) / dt
        state["trace_previous_speed"] = current_speed
        state["trace_previous_frame"] = carla_frame

        longitudinal_acceleration = None
        try:
            acceleration = vehicle.get_acceleration()
            yaw = math.radians(current_transform.rotation.yaw)
            longitudinal_acceleration = (
                float(acceleration.x) * math.cos(yaw)
                + float(acceleration.y) * math.sin(yaw)
            )
        except Exception:
            pass

        applied_throttle = None
        applied_brake = None
        applied_steer = None
        try:
            applied_control = vehicle.get_control()
            applied_throttle = self._as_finite_float(
                getattr(applied_control, "throttle", None)
            )
            applied_brake = self._as_finite_float(getattr(applied_control, "brake", None))
            applied_steer = self._as_finite_float(getattr(applied_control, "steer", None))
        except Exception:
            pass

        trace = {
            "actor_id": veh_id,
            "carla_frame": carla_frame,
            "simulation_time": self.terasim_states.get("simulation_time"),
            "sumo_x": self._as_finite_float(veh_info.get("x")),
            "sumo_y": self._as_finite_float(veh_info.get("y")),
            "sumo_lane_id": veh_info.get("lane_id"),
            "sumo_lane_position": self._as_finite_float(
                veh_info.get("lane_position")
            ),
            "sumo_lateral_offset": self._as_finite_float(
                veh_info.get("lateral_offset")
            ),
            "sumo_lookahead_x": self._as_finite_float(veh_info.get("lookahead_x")),
            "sumo_lookahead_y": self._as_finite_float(veh_info.get("lookahead_y")),
            "lookahead_origin_x": self._as_finite_float(
                veh_info.get("lookahead_origin_x")
            ),
            "lookahead_origin_y": self._as_finite_float(
                veh_info.get("lookahead_origin_y")
            ),
            "carla_x": self._as_finite_float(current_transform.location.x),
            "carla_y": self._as_finite_float(current_transform.location.y),
            "carla_yaw": self._as_finite_float(current_transform.rotation.yaw),
            "sumo_desired_speed": self._as_finite_float(veh_info.get("sumo_desired_speed")),
            "sumo_reported_acceleration": self._as_finite_float(veh_info.get("acceleration")),
            "feedback_observed_speed": self._as_finite_float(veh_info.get("feedback_observed_speed")),
            "sumo_requested_acceleration": state.get("sumo_requested_acceleration"),
            "sumo_emergency_decel": state.get("sumo_emergency_decel"),
            "longitudinal_position_error": state.get(
                "longitudinal_position_error"
            ),
            "longitudinal_velocity_error": state.get(
                "longitudinal_velocity_error"
            ),
            "ackermann_target_speed": target_speed,
            "ackermann_target_acceleration": target_acceleration,
            "control_mode": control_mode,
            "emergency_brake_active": bool(
                state.get("emergency_brake_active", False)
            ),
            "emergency_brake_release_ticks": int(
                state.get("emergency_brake_release_ticks", 0)
            ),
            "commanded_throttle": self._as_finite_float(commanded_throttle),
            "commanded_brake": self._as_finite_float(commanded_brake),
            "commanded_steer": self._as_finite_float(commanded_steer),
            "carla_speed": current_speed,
            "carla_speed_delta_acceleration": speed_delta_acceleration,
            "carla_longitudinal_acceleration": longitudinal_acceleration,
            "carla_applied_throttle": applied_throttle,
            "carla_applied_brake": applied_brake,
            "carla_applied_steer": applied_steer,
            "position_error": position_error,
            "lookahead_distance": self._as_finite_float(
                veh_info.get("lookahead_distance")
            ),
            "lookahead_heading_change": self._as_finite_float(
                veh_info.get("lookahead_heading_change")
            ),
            "lookahead_lane_change_blend": self._as_finite_float(
                veh_info.get("lookahead_lane_change_blend")
            ),
            "sumo_lateral_speed": self._as_finite_float(
                veh_info.get("lateral_speed")
            ),
            "feedback_position_skipped_for_lane_change": bool(
                veh_info.get("feedback_position_skipped_for_lane_change", False)
            ),
            "wheel_base_m": self._as_finite_float(state.get("wheel_base_m")),
            "rear_axle_local_x_m": self._as_finite_float(
                state.get("rear_axle_local_x_m")
            ),
            "front_bumper_local_x_m": self._as_finite_float(
                state.get("front_bumper_local_x_m")
            ),
            "front_bumper_from_bounding_box": bool(
                state.get("front_bumper_from_bounding_box", False)
            ),
            "geometry_from_physics": bool(state.get("geometry_from_physics", False)),
            "pure_pursuit_raw_steer": self._as_finite_float(
                getattr(control_values, "raw_steer", None)
            ),
            "pure_pursuit_clamped_steer": self._as_finite_float(
                getattr(control_values, "clamped_steer", None)
            ),
            "pure_pursuit_command_steer": self._as_finite_float(
                getattr(control_values, "steer", None)
            ),
            "lookahead_local_x": self._as_finite_float(
                getattr(control_values, "lookahead_local_x", None)
            ),
            "lookahead_local_y": self._as_finite_float(
                getattr(control_values, "lookahead_local_y", None)
            ),
            "control_point_x": self._as_finite_float(
                getattr(control_values, "control_point_x", None)
            ),
            "control_point_y": self._as_finite_float(
                getattr(control_values, "control_point_y", None)
            ),
            "feedback_unhealthy": feedback_unhealthy,
            "target_behind": target_behind,
        }
        print("AckermannControlTrace " + json.dumps(trace, sort_keys=True), flush=True)

    def _queue_actor_ackermann_control(
        self, actor, control, ackermann_batch=None, direct_vehicle_control=False
    ):
        command_name = (
            "ApplyVehicleControl"
            if direct_vehicle_control
            else "ApplyVehicleAckermannControl"
        )
        apply_command = getattr(carla.command, command_name, None)
        if ackermann_batch is not None and apply_command is not None:
            ackermann_batch.append(apply_command(actor.id, control))
            return
        if direct_vehicle_control:
            actor.apply_control(control)
        else:
            actor.apply_ackermann_control(control)

    def _flush_actor_ackermann_batch(self, ackermann_batch):
        if not ackermann_batch:
            return []
        return self.client.apply_batch_sync(ackermann_batch, False)

    def _prune_ackermann_actor_state(self, vehicle_ids):
        active_vehicle_ids = set(vehicle_ids)
        for veh_id in list(self._ackermann_actor_state):
            if veh_id not in active_vehicle_ids:
                self._ackermann_actor_state.pop(veh_id, None)

    def _prune_ackermann_feedback_state(self, actor_ids):
        active_actor_ids = set(actor_ids)
        for actor_id in list(self._ackermann_feedback_state):
            if actor_id not in active_actor_ids:
                self._ackermann_feedback_state.pop(actor_id, None)

    # Elevated spawn height to avoid collision with OpenDRIVE-generated road geometry
    # (guardrails, curbs, barriers). After spawn, correct transform is set immediately.
    SPAWN_Z_CLEARANCE = 5.0
    ACKERMANN_SPAWN_STABILITY_TICKS = 3
    ACKERMANN_SPAWN_MAX_Z_ERROR = 0.5
    ACKERMANN_SPAWN_MAX_SPEED_ERROR = 3.0
    ACKERMANN_SPAWN_MAX_VERTICAL_SPEED = 1.0
    ACKERMANN_SPAWN_MAX_TILT_ERROR = 10.0
    SPAWN_RETRY_FRAMES = 10
    SPAWN_RETRY_DISTANCE = 5.0

    @staticmethod
    def _spawn_failure_key(actor_type, actor_id):
        return actor_type, actor_id

    def _fresh_blueprint(self, blueprint):
        try:
            return self.world.get_blueprint_library().find(blueprint.id)
        except Exception:
            return blueprint

    def _select_vehicle_blueprint(self, veh_id, veh_info):
        if "BIKE" in veh_info["type"]:
            blueprint = self._random.choice(self.bike_blueprints)
        elif "MOTOR" in veh_info["type"]:
            blueprint = self._random.choice(self.motor_blueprints)
        elif "POLICE" in veh_info["type"]:
            blueprint = self._random.choice(self.police_car_blueprints)
        else:
            blueprint = self._random.choice(self.vehicle_blueprints)
        blueprint = self._fresh_blueprint(blueprint)
        blueprint.set_attribute("role_name", veh_id)
        if veh_id == AV_SUMO_ID:
            blueprint.set_attribute("color", "255, 0, 0")
        else:
            blueprint.set_attribute("color", "0, 102, 204")
        return blueprint

    def _select_vru_blueprint(self, vru_id, vru_info):
        if "BIKE" in vru_info["type"]:
            blueprint = self._random.choice(self.bike_blueprints)
        elif "MOTOR" in vru_info["type"]:
            blueprint = self._random.choice(self.motor_blueprints)
        else:
            blueprint = self._random.choice(self.pedestrian_blueprints)
        blueprint = self._fresh_blueprint(blueprint)
        blueprint.set_attribute("role_name", vru_id)
        return blueprint

    @staticmethod
    def _empty_spawn_batch_stats():
        return {
            "spawn_calls": 0,
            "spawn_total": 0.0,
            "spawn_max": 0.0,
            "spawn_success": 0,
            "spawn_failed": 0,
            "apply_batch_sync_time": 0.0,
            "apply_batch_sync_max": 0.0,
            "apply_batch_sync_success_time": 0.0,
            "apply_batch_sync_failed_time": 0.0,
        }

    def _queue_actor_spawn(self, spawn_requests, request):
        if self.batch_spawn_enabled and spawn_requests is not None:
            spawn_requests.append(request)
            return True
        return False

    def _flush_actor_spawn_batch(self, spawn_requests, transform_batch=None):
        stats = self._empty_spawn_batch_stats()
        if not spawn_requests:
            return stats

        commands = [
            carla.command.SpawnActor(request["blueprint"], request["spawn_transform"]).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False)
            )
            for request in spawn_requests
        ]

        total_start = time.perf_counter()
        apply_start = time.perf_counter()
        # due_tick_cue=False: with True, in synchronous mode this call blocks until the
        # next world tick. Spawns happen every time a vehicle enters the sync radius, so
        # the blocking variant locks the whole co-sim loop to 2 CARLA ticks per step
        # (measured: client wait p50 ~70ms vs the 51ms tick period). The responses
        # (actor ids) are still returned without the tick; an actor that is not yet
        # resolvable falls into the existing spawn-failure retry path.
        responses = self.client.apply_batch_sync(commands, False)
        apply_elapsed = time.perf_counter() - apply_start

        stats["spawn_calls"] = len(spawn_requests)
        stats["apply_batch_sync_time"] = apply_elapsed
        stats["apply_batch_sync_max"] = apply_elapsed

        response_count = len(responses)
        success_count = sum(1 for response in responses if not response.error)
        failed_count = response_count - success_count
        stats["spawn_success"] = success_count
        stats["spawn_failed"] = failed_count
        if response_count:
            stats["apply_batch_sync_success_time"] = apply_elapsed * success_count / response_count
            stats["apply_batch_sync_failed_time"] = apply_elapsed * failed_count / response_count

        for request, response in zip(spawn_requests, responses):
            actor_type = request["actor_type"]
            actor_id = request["actor_id"]
            if response.error:
                log_spawn_actor_failure(
                    self.world,
                    request["blueprint"],
                    request["spawn_transform"],
                    actor_id,
                    response.error,
                )
                self._record_spawn_failure(
                    actor_type, actor_id, request["sumo_location"], request["current_frame"]
                )
                continue

            actor = self.world.get_actor(response.actor_id)
            if actor is None:
                target_index = request.get("actor_index")
                index_kind = (
                    "vehicle"
                    if target_index is self._vehicle_actor_index
                    else "pedestrian"
                )
                self._pending_actor_index_entries[actor_id] = (
                    index_kind,
                    response.actor_id,
                )
                self._record_spawn_failure(
                    actor_type, actor_id, request["sumo_location"], request["current_frame"]
                )
                continue

            self._clear_spawn_failure(actor_type, actor_id)
            self._pending_actor_index_entries.pop(actor_id, None)
            actor_index = request.get("actor_index")
            if actor_index is not None:
                actor_index[actor_id] = actor
            if actor_type == "vehicle":
                self._ensure_collision_sensor(actor, actor_id)
            if request.get("enable_physics_after_spawn", False):
                self._ackermann_actor_state.pop(actor_id, None)
                actor_state = self._ackermann_actor_state.setdefault(actor_id, {})
                self._initialize_ackermann_actor_geometry(
                    actor, actor_id, actor_state
                )
                post_spawn_transform = self._sumo_front_to_carla_transform(
                    request["sumo_location"],
                    request["sumo_rotation"],
                    request["shape"],
                    request["post_spawn_offset"],
                    actor_state.get("front_bumper_local_x_m"),
                )
                self._prepare_ackermann_actor_physics(
                    actor,
                    actor_id,
                    request["actor_info"].get("speed"),
                    post_spawn_transform,
                    request["spawn_transform"],
                )
            else:
                self._queue_actor_transform(actor, request["post_spawn_transform"], transform_batch)
            if actor_type == "vru":
                self._apply_vru_walker_control(request["actor_info"], request["sumo_angle"], actor)

        stats["spawn_total"] = time.perf_counter() - total_start
        stats["spawn_max"] = stats["spawn_total"]
        return stats

    def _apply_vru_walker_control(self, vru_info, sumo_angle, pedestrian):
        if self._vru_uses_vehicle_blueprint(vru_info):
            return
        radians = math.radians(90 - sumo_angle)
        orientation = math.atan2(math.sin(radians), math.cos(radians))
        direction_x, direction_y = math.cos(orientation), math.sin(orientation)
        walker_control = carla.WalkerControl(
            direction=carla.Vector3D(direction_x, direction_y, 0),
            speed=vru_info["speed"],
        )
        try:
            pedestrian.apply_control(walker_control)
        except Exception:
            pass

    def _queue_actor_transform(self, actor, transform, transform_batch=None):
        """Queue or apply a CARLA actor transform according to batching settings."""
        if self.batch_transform_enabled and transform_batch is not None:
            transform_batch.append(carla.command.ApplyTransform(actor.id, transform))
            return
        actor.set_transform(transform)

    def _flush_actor_transform_batch(self, transform_batch):
        """Apply queued actor transforms in one CARLA batch call."""
        if not transform_batch:
            return []
        return self.client.apply_batch_sync(transform_batch, False)

    def _build_actor_role_indexes(self):
        """Return persistent role_name indexes, scanning CARLA only once."""
        if (
            self._vehicle_actor_index is not None
            and self._pedestrian_actor_index is not None
        ):
            self._refresh_persistent_actor_indexes()
            return self._vehicle_actor_index, self._pedestrian_actor_index

        vehicle_actor_index = {}
        pedestrian_actor_index = {}
        actors = self.world.get_actors()
        world_actor_count = 0
        for actor in actors:
            world_actor_count += 1
            role_name = actor.attributes.get("role_name")
            if not role_name:
                continue
            if actor.type_id.startswith("vehicle."):
                vehicle_actor_index.setdefault(role_name, actor)
            elif actor.type_id.startswith("walker.pedestrian."):
                pedestrian_actor_index.setdefault(role_name, actor)
        self._last_actor_index_world_actor_count = world_actor_count
        self._last_actor_index_vehicle_actor_count = len(vehicle_actor_index)
        self._last_actor_index_pedestrian_actor_count = len(pedestrian_actor_index)
        self._vehicle_actor_index = vehicle_actor_index
        self._pedestrian_actor_index = pedestrian_actor_index
        return vehicle_actor_index, pedestrian_actor_index

    def _refresh_persistent_actor_indexes(self):
        """Resolve deferred spawns and discard actors CARLA already destroyed."""
        for actor_index in (
            self._vehicle_actor_index or {},
            self._pedestrian_actor_index or {},
        ):
            for role_name, actor in list(actor_index.items()):
                try:
                    alive = actor is not None and actor.is_alive
                except Exception:
                    alive = False
                if not alive:
                    actor_index.pop(role_name, None)

        for role_name, (index_kind, carla_id) in list(
            self._pending_actor_index_entries.items()
        ):
            actor = self.world.get_actor(carla_id)
            if actor is None:
                continue
            if index_kind == "vehicle":
                self._vehicle_actor_index[role_name] = actor
            else:
                self._pedestrian_actor_index[role_name] = actor
            self._pending_actor_index_entries.pop(role_name, None)

    @staticmethod
    def _vru_uses_vehicle_blueprint(vru_info):
        return "BIKE" in vru_info["type"] or "MOTOR" in vru_info["type"]

    def _should_retry_spawn(self, actor_type, actor_id, sumo_location, current_frame):
        failure = self._spawn_failures.get(self._spawn_failure_key(actor_type, actor_id))
        if failure is None:
            return True
        if failure.get("abandoned"):
            return False
        if actor_id == AV_SUMO_ID:
            return True

        dx = sumo_location[0] - failure["x"]
        dy = sumo_location[1] - failure["y"]
        if dx * dx + dy * dy >= self.SPAWN_RETRY_DISTANCE * self.SPAWN_RETRY_DISTANCE:
            return True

        next_retry_time = failure.get("next_retry_time")
        if next_retry_time is not None:
            return time.monotonic() >= next_retry_time
        return current_frame >= failure.get("next_retry_frame", current_frame)

    def _record_spawn_failure(self, actor_type, actor_id, sumo_location, current_frame):
        key = self._spawn_failure_key(actor_type, actor_id)
        previous = self._spawn_failures.get(key, {})
        failures = previous.get("failures", 0) + 1
        exponent = min(failures - 1, 30)
        delay = min(
            self.spawn_failure_backoff_max_seconds,
            self.spawn_failure_backoff_seconds * (2 ** exponent),
        )
        abandoned = failures >= max(1, int(getattr(self, "spawn_max_attempts", 3)))
        self._spawn_failures[key] = {
            "failures": failures,
            "abandoned": abandoned,
            "next_retry_frame": current_frame + self.SPAWN_RETRY_FRAMES,
            "next_retry_time": time.monotonic() + delay,
            "x": sumo_location[0],
            "y": sumo_location[1],
        }
        if abandoned:
            print(
                f"Warning: abandoning CARLA {actor_type} spawn for {actor_id!r} "
                f"after {failures} failed attempts. SUMO simulation will continue.",
                flush=True,
            )

    def _mark_spawn_abandoned(self, actor_type, actor_id, failures, reason):
        self._spawn_failures[self._spawn_failure_key(actor_type, actor_id)] = {
            "failures": int(failures),
            "abandoned": True,
            "reason": str(reason),
        }

    def _is_spawn_abandoned(self, actor_type, actor_id):
        failure = self._spawn_failures.get(self._spawn_failure_key(actor_type, actor_id))
        return bool(failure and failure.get("abandoned"))

    def _clear_spawn_failure(self, actor_type, actor_id):
        self._spawn_failures.pop(self._spawn_failure_key(actor_type, actor_id), None)

    @staticmethod
    def _as_finite_float(value):
        if isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if math.isfinite(number):
            return number
        return None

    def _warn_invalid_sumo_location_once(self, actor_type, actor_id, raw_location):
        key = (actor_type, actor_id)
        if key in self._invalid_location_warnings:
            return
        self._invalid_location_warnings.add(key)
        print(
            f"Warning: {actor_type} {actor_id!r} has invalid SUMO location "
            f"{raw_location}; skipping this tick.",
            flush=True,
        )

    def _resolve_sumo_location(self, actor_type, actor_id, actor_info, prefer_lane_relative=False):
        use_reconstructed = (
            prefer_lane_relative
            and bool(actor_info.get("reconstructed_position_valid", False))
        )
        if use_reconstructed:
            raw_location = {
                "x": actor_info.get("reconstructed_x"),
                "y": actor_info.get("reconstructed_y"),
                "z": actor_info.get("reconstructed_z"),
            }
        else:
            raw_location = {
                "x": actor_info.get("x"),
                "y": actor_info.get("y"),
                "z": actor_info.get("z"),
            }
        sumo_location = [
            self._as_finite_float(raw_location["x"]),
            self._as_finite_float(raw_location["y"]),
            self._as_finite_float(raw_location["z"]),
        ]
        if use_reconstructed and any(value is None for value in sumo_location):
            raw_location = {
                "x": actor_info.get("x"),
                "y": actor_info.get("y"),
                "z": actor_info.get("z"),
            }
            sumo_location = [
                self._as_finite_float(raw_location["x"]),
                self._as_finite_float(raw_location["y"]),
                self._as_finite_float(raw_location["z"]),
            ]
        if any(value is None for value in sumo_location):
            self._warn_invalid_sumo_location_once(actor_type, actor_id, raw_location)
            return None
        return sumo_location

    def _warn_missing_sumo_angle_once(self, actor_type, actor_id, action):
        key = (actor_type, actor_id, action)
        if key in self._missing_angle_warnings:
            return
        self._missing_angle_warnings.add(key)
        print(
            f"Warning: {actor_type} {actor_id!r} has missing sumo_angle; {action}.",
            flush=True,
        )

    def _resolve_sumo_angle(self, actor_type, actor_id, actor_info, carla_actor=None):
        angle = self._as_finite_float(actor_info.get("sumo_angle"))
        if angle is not None:
            return angle

        orientation = self._as_finite_float(actor_info.get("orientation"))
        if orientation is not None:
            self._warn_missing_sumo_angle_once(actor_type, actor_id, "using orientation fallback")
            return (90.0 - math.degrees(orientation)) % 360.0

        if carla_actor is not None:
            self._warn_missing_sumo_angle_once(actor_type, actor_id, "using previous CARLA yaw fallback")
            return (carla_actor.get_transform().rotation.yaw + 90.0) % 360.0

        self._warn_missing_sumo_angle_once(actor_type, actor_id, "skipping this tick")
        return None

    def _prune_spawn_failures(self, vehicle_ids, vru_ids):
        active_keys = {
            self._spawn_failure_key("vehicle", vehicle_id) for vehicle_id in vehicle_ids
        }
        active_keys.update(
            self._spawn_failure_key("vru", vru_id) for vru_id in vru_ids
        )
        for key in list(self._spawn_failures):
            if key not in active_keys:
                self._spawn_failures.pop(key, None)

    def _process_vehicle(
        self,
        veh_id,
        veh_info,
        cosim_id_record,
        carla_actor=None,
        actor_index=None,
        current_frame=None,
        transform_batch=None,
        ackermann_batch=None,
        spawn_requests=None,
    ):
        """Process a vehicle actor."""
        if self._is_spawn_abandoned("vehicle", veh_id):
            return
        cosim_id_record.add(veh_id)

        sumo_location = self._resolve_sumo_location(
            "vehicle",
            veh_id,
            veh_info,
            prefer_lane_relative=self.use_lane_relative_position,
        )
        if sumo_location is None:
            return
        sumo_angle = self._resolve_sumo_angle("vehicle", veh_id, veh_info, carla_actor)
        if sumo_angle is None:
            return
        sumo_slope = self._as_finite_float(veh_info.get("sumo_slope")) or 0.0
        sumo_rotation = [sumo_slope, sumo_angle, 0.0]
        shape = [veh_info["length"], veh_info["width"], veh_info["height"]]
        uses_ackermann_physics = self._uses_ackermann_physics(veh_id)

        vehicle = carla_actor
        if vehicle is not None:
            self._ensure_collision_sensor(vehicle, veh_id)
        if vehicle is None:
            if current_frame is None:
                current_frame = self.world.get_snapshot().frame
            if not self._should_retry_spawn("vehicle", veh_id, sumo_location, current_frame):
                return
            blueprint = self._select_vehicle_blueprint(veh_id, veh_info)
            # Spawn elevated to avoid collision with road geometry, then set correct transform
            sumo_offset = self._get_carla_offset(sumo_location, self.spawn_z_clearance)
            spawn_transform = (
                self._sumo_front_to_carla_transform(
                    sumo_location, sumo_rotation, shape, sumo_offset
                )
                if uses_ackermann_physics
                else sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            )
            sumo_offset_correct = self._get_carla_offset(sumo_location, 0.0)
            carla_trasform = (
                self._sumo_front_to_carla_transform(
                    sumo_location, sumo_rotation, shape, sumo_offset_correct
                )
                if uses_ackermann_physics
                else sumo_to_carla(
                    sumo_location, sumo_rotation, shape, sumo_offset_correct
                )
            )
            if self._queue_actor_spawn(
                spawn_requests,
                {
                    "actor_type": "vehicle",
                    "actor_id": veh_id,
                    "actor_info": veh_info,
                    "blueprint": blueprint,
                    "spawn_transform": spawn_transform,
                    "post_spawn_transform": carla_trasform,
                    "sumo_location": sumo_location,
                    "sumo_rotation": sumo_rotation,
                    "shape": shape,
                    "post_spawn_offset": sumo_offset_correct,
                    "current_frame": current_frame,
                    "actor_index": actor_index,
                    "sumo_angle": sumo_angle,
                    "enable_physics_after_spawn": uses_ackermann_physics,
                },
            ):
                return
            carla_id = spawn_actor(
                self.client,
                blueprint,
                spawn_transform,
                world=self.world,
                actor_role=veh_id,
            )
            if carla_id > 0:
                self._clear_spawn_failure("vehicle", veh_id)
                vehicle = self.world.get_actor(carla_id)
                if vehicle is None:
                    self._record_spawn_failure("vehicle", veh_id, sumo_location, current_frame)
                    return
                if actor_index is not None:
                    actor_index[veh_id] = vehicle
                self._ensure_collision_sensor(vehicle, veh_id)
                if uses_ackermann_physics:
                    self._ackermann_actor_state.pop(veh_id, None)
                    actor_state = self._ackermann_actor_state.setdefault(veh_id, {})
                    self._initialize_ackermann_actor_geometry(
                        vehicle, veh_id, actor_state
                    )
                    carla_trasform = self._sumo_front_to_carla_transform(
                        sumo_location,
                        sumo_rotation,
                        shape,
                        sumo_offset_correct,
                        actor_state.get("front_bumper_local_x_m"),
                    )
                    self._prepare_ackermann_actor_physics(
                        vehicle,
                        veh_id,
                        veh_info.get("speed"),
                        carla_trasform,
                        spawn_transform,
                    )
                else:
                    # Immediately set the correct road-level transform.
                    vehicle.set_transform(carla_trasform)
            else:
                self._record_spawn_failure("vehicle", veh_id, sumo_location, current_frame)
        else:
            sumo_offset = self._get_carla_offset(sumo_location, 0.0)
            carla_trasform = (
                self._sumo_front_to_carla_transform(
                    sumo_location, sumo_rotation, shape, sumo_offset
                )
                if uses_ackermann_physics
                else sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            )
            if uses_ackermann_physics:
                physics_ready = self._ensure_ackermann_actor_physics(
                    vehicle,
                    veh_id,
                    veh_info.get("speed"),
                    carla_trasform,
                )
                if not physics_ready:
                    return
                actor_state = self._ackermann_actor_state.setdefault(veh_id, {})
                carla_trasform = self._sumo_front_to_carla_transform(
                    sumo_location,
                    sumo_rotation,
                    shape,
                    sumo_offset,
                    actor_state.get("front_bumper_local_x_m"),
                )
                control = self._build_ackermann_control(
                    veh_id,
                    veh_info,
                    vehicle,
                    sumo_location,
                    sumo_angle,
                    carla_trasform,
                )
                self._queue_actor_ackermann_control(
                    vehicle,
                    control,
                    ackermann_batch,
                    direct_vehicle_control=(
                        actor_state.get("control_mode") == "emergency_brake"
                    ),
                )
            else:
                self._ensure_actor_teleport_mode(vehicle, veh_id)
                self._queue_actor_transform(vehicle, carla_trasform, transform_batch)

    def _process_vru(
        self,
        vru_id,
        vru_info,
        cosim_id_record,
        carla_actor=None,
        actor_index=None,
        current_frame=None,
        transform_batch=None,
        spawn_requests=None,
    ):
        """Process a pedestrian actor."""
        cosim_id_record.add(vru_id)

        sumo_location = self._resolve_sumo_location("vru", vru_id, vru_info)
        if sumo_location is None:
            return
        sumo_angle = self._resolve_sumo_angle("vru", vru_id, vru_info, carla_actor)
        if sumo_angle is None:
            return
        sumo_rotation = [0.0, sumo_angle, 0.0]
        shape = [vru_info["length"], vru_info["width"], vru_info["height"]]

        pedestrian = carla_actor
        carla_id = pedestrian.id if pedestrian is not None else -1
        if pedestrian is None:
            if current_frame is None:
                current_frame = self.world.get_snapshot().frame
            if not self._should_retry_spawn("vru", vru_id, sumo_location, current_frame):
                return
            blueprint = self._select_vru_blueprint(vru_id, vru_info)
            # Spawn elevated to avoid collision with road geometry
            sumo_offset = self._get_carla_offset(sumo_location, self.spawn_z_clearance)
            spawn_transform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            z_off = 0.0 if "BIKE" in vru_info["type"] else shape[2] / 2.0
            sumo_offset_correct = self._get_carla_offset(sumo_location, z_off)
            carla_trasform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset_correct)
            if self._queue_actor_spawn(
                spawn_requests,
                {
                    "actor_type": "vru",
                    "actor_id": vru_id,
                    "actor_info": vru_info,
                    "blueprint": blueprint,
                    "spawn_transform": spawn_transform,
                    "post_spawn_transform": carla_trasform,
                    "sumo_location": sumo_location,
                    "current_frame": current_frame,
                    "actor_index": actor_index,
                    "sumo_angle": sumo_angle,
                },
            ):
                return
            carla_id = spawn_actor(
                self.client,
                blueprint,
                spawn_transform,
                world=self.world,
                actor_role=vru_id,
            )
            if carla_id > 0:
                self._clear_spawn_failure("vru", vru_id)
                pedestrian = self.world.get_actor(carla_id)
                if pedestrian is None:
                    self._record_spawn_failure("vru", vru_id, sumo_location, current_frame)
                    return
                if actor_index is not None:
                    actor_index[vru_id] = pedestrian
                pedestrian.set_transform(carla_trasform)
            else:
                self._record_spawn_failure("vru", vru_id, sumo_location, current_frame)
        else:
            z_off = 0.0 if "BIKE" in vru_info["type"] else shape[2] / 2.0
            sumo_offset = self._get_carla_offset(sumo_location, z_off)
            carla_trasform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            self._queue_actor_transform(pedestrian, carla_trasform, transform_batch)

        if carla_id > 0:
            self._apply_vru_walker_control(vru_info, sumo_angle, pedestrian)

    def _cleanup_actors(self, actor_type, pattern, cosim_id_record):
        """Batch-destroy stale indexed actors without scanning the CARLA world."""
        # Protect ego (and any other role names passed via protected_roles). In 3-cosim the
        # psim ego has role_name "ego_vehicle", which must not be destroyed as a stale actor.
        protected = set(getattr(self.args, "protected_roles", None) or ["AV"])
        if actor_type == "vehicle":
            actor_index = self._vehicle_actor_index or {}
        else:
            actor_index = self._pedestrian_actor_index or {}

        stale_roles = [
            role_name
            for role_name in actor_index
            if role_name not in cosim_id_record and role_name not in protected
        ]
        stale_actors = [
            actor_index[role_name]
            for role_name in stale_roles
            if actor_index.get(role_name) is not None
        ]
        stale_pending_roles = [
            role_name
            for role_name, (pending_type, _carla_id) in self._pending_actor_index_entries.items()
            if pending_type == actor_type
            and role_name not in cosim_id_record
            and role_name not in protected
        ]
        stale_pending_ids = [
            self._pending_actor_index_entries[role_name][1]
            for role_name in stale_pending_roles
        ]
        if actor_type == "vehicle":
            for role_name in stale_roles:
                self._remove_collision_sensor(role_name)
        if stale_actors or stale_pending_ids:
            destroy_commands = [
                carla.command.DestroyActor(actor.id) for actor in stale_actors
            ]
            destroy_commands.extend(
                carla.command.DestroyActor(actor_id) for actor_id in stale_pending_ids
            )
            try:
                self.client.apply_batch_sync(destroy_commands, False)
            except Exception as exc:
                print(
                    f"Warning: CARLA stale {actor_type} batch cleanup failed: {exc}",
                    flush=True,
                )
                for actor in stale_actors:
                    try:
                        actor.destroy()
                    except Exception:
                        pass
        for role_name in stale_roles:
            actor_index.pop(role_name, None)
        for role_name in stale_pending_roles:
            self._pending_actor_index_entries.pop(role_name, None)

    def close(self):
        """
        Cleans synchronization and resets the simulation settings.
        """
        def wait_for_cleanup_tick(stage):
            try:
                self.world.wait_for_tick(10.0)
            except Exception as exc:
                print(f"Warning: CARLA cleanup tick failed during {stage}: {exc}")

        if not getattr(self.args, "passive_tick", False):
            # Configuring carla simulation in async mode.
            # Skipped in 3-cosim passive mode: the psim bridge owns synchronous_mode, and
            # resetting it here would break psim's sync loop.
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            wait_for_cleanup_tick("sync-to-async transition")
        
        self._shutdown_collision_sensors()

        # Destroy actors. In 3-cosim passive mode, keep ego (protected_roles) and clear only the
        # SUMO-spawned background vehicles/pedestrians; otherwise destroy everything.
        if getattr(self.args, "passive_tick", False):
            protected = getattr(self.args, "protected_roles", None) or ["AV"]
            for actor in self.world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") not in protected:
                    actor.destroy()
            for actor in self.world.get_actors().filter("walker.*"):
                actor.destroy()
        else:
            destroy_all_actors(self.world)
            wait_for_cleanup_tick("first actor cleanup")
            destroy_all_actors(self.world)
            wait_for_cleanup_tick("final actor cleanup")

        # stop TeraSim
        if self.direct_link is not None:
            self.direct_link.stop()
            self.direct_link.close()
        else:
            stop_terasim(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])
