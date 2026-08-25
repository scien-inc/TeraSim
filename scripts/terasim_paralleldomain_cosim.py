"""
TeraSim - Parallel Domain Co-simulation Script

This script enables co-simulation between TeraSim and Parallel Domain's
photorealistic rendering platform.

Prerequisites:
    1. Install Parallel Domain SDK:
       pip install paralleldomain

    2. Configure environment variables in .env file (see .env.example):
       - PD_ORG: Your organization name
       - PD_API_KEY: Your API key
       - PD_CERT_PATH: Path to your certificate file
       - PD_ENV: Environment (prod/dev)

    3. Remove personFlow from Mcity route file:
       The current Mcity PD replica does not support sidewalks, so pedestrian
       flows must be removed from examples/maps/Mcity/mcity.rou.xml before
       running the co-simulation.
"""

import json
import logging
import os
from enum import Enum
from typing import Optional, Tuple

import requests
import time

import logging
import random
from typing import Literal, Optional, Union
from uuid import uuid4

import numpy as np
from dotenv import load_dotenv

from paralleldomain import sdk

# Load environment variables from .env file
load_dotenv()

# Agent type enumeration for tracking different agent types
class AgentType(Enum):
    VEHICLE = "vehicle"
    BICYCLE = "bicycle"
    PEDESTRIAN = "pedestrian"


# SUMO vehicle type -> PD asset category mapping
SUMO_TO_PD_CATEGORY = {
    # Vehicle types
    "NDE_URBAN": "sedan",
    "NDE_HIGHWAY": "sedan",
    "car": "sedan",
    "passenger": "sedan",
    "taxi": "sedan",
    "suv": "suv",
    "truck": "truck",
    "trailer": "truck",
    "bus": "bus",
    "coach": "bus",
    # Bicycle types
    "bike": "bicycle",
    "bicycle": "bicycle",
    "DEFAULT_BIKETYPE": "bicycle",
    # Pedestrian types
    "DEFAULT_PEDTYPE": "pedestrian",
    "pedestrian": "pedestrian",
}

# PD asset category -> list of available assets for random selection
PD_ASSETS_BY_CATEGORY = {
    "sedan": [
        "fullsize_sedan_01", "fullsize_sedan_02", "fullsize_sedan_03",
        "midsize_sedan_01", "midsize_sedan_02", "midsize_sedan_03", "midsize_sedan_04"
    ],
    "suv": [
        "suv_compact_01", "suv_compact_02_ev",
        "suv_medium_01", "suv_medium_02",
        "suv_large_01", "suv_large_02"
    ],
    "pickup": [
        "pickup_compact_01",
        "pickup_medium_01", "pickup_medium_02",
        "pickup_large_01_pickup"
    ],
    "truck": ["truck_semi_01", "truck_semi_02"],
    "bus": ["bus_city_01", "bus_school_01", "bus_school_02", "bus_coach_01"],
    "bicycle": ["bicycle_city_01_a", "bicycle_mountainbike_01", "bicycle_rental_bike_01_a"],
    "pedestrian": [
        "char_adanna_007", "char_alexandra_003", "char_henry_004",
        "char_sophia", "char_scott_005", "char_hannah_001",
        "char_jason_001", "char_felice_005", "char_eric_004"
    ]
}
    
CLIENT_SIMULATOR_TICKS_PER_SECOND = 80  # The PD Simulator runs at a fixed rate of 100 ticks per second

# PD configuration is now loaded from .env file via dotenv
# Required environment variables: PD_ORG, PD_API_KEY, PD_CERT_PATH, PD_ENV

# Module logger
logger = logging.getLogger(__name__)
if not logger.handlers:
    logging.basicConfig(level=logging.INFO)

class TeraSimClient:
    """Simple client for interacting with the local TeraSim service.

    Usage:
        client = TeraSimClient("http://localhost:8000")
        sim_id = client.start_simulation("examples/simulation_config.yaml")
    """

    def __init__(
            self,
            base_url: str,
            location: str,
            lighting: str,
            seed: int,
            simulation_length_s: float,
            framerate_fps: float,
            instance_name: Optional[str] = None,
            control_input_time_delta: float = 0.5,  # How often a control input is provided to the ego agent
            visualize: bool = False,
            filter_center_id: Optional[str] = "AV",  # Center vehicle ID for filtering nearby agents
            filter_radius: Optional[float] = 100.0,  # Radius in meters to filter agents around center
        ):
        self.base_url = base_url.rstrip("/")
        # in-memory last started simulation id
        self.simulation_id: Optional[str] = None
        # store simulation states
        self.sim_states: Optional[dict] = None
        # parallel domain client (lazy init)
        self._world = None
        self._location = location
        self._lighting = lighting
        self._seed = seed
        self._instance_name = instance_name

        # The PD Simulator runs at a fixed rate of 0.01 seconds per simulation step (100 fps). Because of this, the
        # simulation times at which you can render a frame through the PD Renderer must be divisible by this
        # time delta. For example, you can capture a frame every 0.04 seconds in simulation time (25 fps).
        # Below, we calculate the number of simulation timesteps (at 100 fps) and the capture rate of the scenario,
        # based on the input parameters.  We also calculate the number of timesteps (at 100 fps) between the control
        # inputs provided to the ego
        self._number_of_ticks = int(round(simulation_length_s, 2) * CLIENT_SIMULATOR_TICKS_PER_SECOND)
        # self._capture_rate = int(round(1 / framerate_fps, 2) * CLIENT_SIMULATOR_TICKS_PER_SECOND)
        self._capture_rate = 1
        self._ticks_between_control_input = int(round(control_input_time_delta, 2) * CLIENT_SIMULATOR_TICKS_PER_SECOND)
        self._rng = random.Random(seed)
        self._visualize = visualize
        self._filter_center_id = filter_center_id
        self._filter_radius = filter_radius

        # Log a warning if the frame rate is different from the requested one because of the 100 fps limitation
        actual_framerate = CLIENT_SIMULATOR_TICKS_PER_SECOND / self._capture_rate
        if actual_framerate != framerate_fps:
            logger.warning(
                f"Requested framerate ({framerate_fps}) fps cannot be exactly replicated as the PD Simulator runs at "
                f"100 fps, will output closest approximation ({actual_framerate}) fps"
            )

        # Set up a simple Sensor Rig
        self._sensor_rig = sdk.SensorRig(
            sensor_configs=[
                sdk.sensors.PinholeCamera(
                    name="Pinhole",
                    height=1080,
                    width=1920,
                    field_of_view=70.0,
                    pose=sdk.Transformation.from_euler_angles(order="xyz", angles=[-90.0, 0.0, 0.0], degrees=True),
                )
            ]
        )

        # Agent tracking maps
        self.agent_id_map = {}      # agent_key -> PD agent ID
        self.agent_type_map = {}    # agent_key -> AgentType enum
        self.agent_asset_map = {}   # agent_key -> asset name (for updates)
        self.agent_last_pose = {}   # agent_key -> (x, y) for speed calculation
        self._cars_poses = {}

    def _post(self, path: str, payload: dict) -> dict:
        """POST helper. `payload` may be None. Optional query `params` may be provided."""
        params = None
        # Support payload passed as dict or None; if a tuple is used to pass params, handle gracefully
        if isinstance(payload, tuple):
            # backward compatibility: allow calling _post(path, (payload, params))
            payload, params = payload
        url = f"{self.base_url}{path}"
        headers = {"Content-Type": "application/json"}
        # logger.info("POST %s", url)
        # logger.debug("Payload: %s", payload)
        resp = requests.post(url, headers=headers, data=json.dumps(payload) if payload is not None else None, params=params)
        # logger.info("Response status: %s", resp.status_code)
        try:
            body = resp.json()
            logger.debug("Response JSON: %s", body)
        except ValueError:
            body = resp.text
            logger.debug("Response text: %s", body)
        resp.raise_for_status()
        return body

    def _get(self, path: str, params: Optional[dict] = None) -> dict:
        """GET helper returning parsed JSON or raising on error."""
        url = f"{self.base_url}{path}"
        # logger.info("GET %s", url)
        resp = requests.get(url, params=params)
        # logger.info("GET response status: %s", resp.status_code)
        # logger.debug("GET response headers: %s", resp.headers)
        resp.raise_for_status()
        try:
            body = resp.json()
            logger.debug("GET response JSON: %s", body)
            return body
        except ValueError:
            text = resp.text
            logger.debug("GET response text: %s", text)
            raise

    # Convenience control methods used by run_simulation
    def tick(self, simulation_id: str) -> dict:
        return self._post(f"/simulation_tick/{simulation_id}", None)

    def control_simulation(self, simulation_id: str, command: str) -> dict:
        return self._post(f"/simulation_control/{simulation_id}", {"command": command})

    def get_state(self, simulation_id: str, center_id: Optional[str] = None, radius: Optional[float] = None) -> dict:
        params = {}
        if center_id is not None:
            params["center_id"] = center_id
        if radius is not None:
            params["radius"] = radius
        return self._get(f"/simulation/{simulation_id}/state", params=params if params else None)

    def get_result(self, simulation_id: str) -> dict:
        return self._get(f"/simulation_result/{simulation_id}")

    def check_simulation_status(self, simulation_id: str) -> dict:
        """Query the service for a simulation's status.

        Calls GET /simulation_status/{simulationId} and returns parsed JSON.
        """
        return self._get(f"/simulation_status/{simulation_id}")

    def _retrieve_rgb(self, timestamp: int, base_path: str = "./output_images/") -> None:
        """
        Retrieve and save RGB images from the rendered frame.
        Images are saved to subdirectories by camera type.
        """
        from PIL import Image

        for i, annotation in enumerate(self._world.get_annotations(annotation_types=sdk.annotations.RGB)):
            camera_type = f"camera_{i}"
            camera_path = os.path.join(base_path, camera_type)
            os.makedirs(camera_path, exist_ok=True)

            image = annotation.image  # numpy array
            pil_image = Image.fromarray(image)
            pil_image.save(f"{camera_path}/rgb_frame_{timestamp:04d}_{i}.png")
            logger.info(f"Saved RGB image for timestamp {timestamp} at {camera_path}/rgb_frame_{timestamp:04d}_{i}.png")

    def _retrieve_lidar_bev(self, timestamp: int, base_path: str = "./output_lidar_bev/") -> None:
        """Render and save LiDAR BEV images from the point cloud annotation.

        PD SDK uses RFU coordinate system:
        - X = Right (positive to the right of the vehicle)
        - Y = Forward (positive in front of the vehicle)
        - Z = Up (positive above the vehicle)

        For BEV (Bird's Eye View), we plot X-Y plane looking down from Z axis.
        """
        import matplotlib.pyplot as plt

        os.makedirs(base_path, exist_ok=True)

        for annotation in self._world.get_annotations(annotation_types=sdk.annotations.PointCloud):
            xyz = annotation.xyz           # (N, 3) in RFU coordinates
            intensity = annotation.intensity.flatten()  # (N,)

            # Log point cloud statistics for debugging
            logger.info(f"Point cloud stats - X: [{xyz[:,0].min():.1f}, {xyz[:,0].max():.1f}], "
                       f"Y: [{xyz[:,1].min():.1f}, {xyz[:,1].max():.1f}], "
                       f"Z: [{xyz[:,2].min():.1f}, {xyz[:,2].max():.1f}], "
                       f"N points: {len(xyz)}")

            # Normalize intensity for better visualization
            intensity_norm = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-6)

            # Create 1:2 vertical BEV image (width 5 inches, height 10 inches)
            fig, ax = plt.subplots(figsize=(5, 10), dpi=100)

            # BEV view: X (right) on horizontal axis, Y (forward) on vertical axis
            # Negate X so that left is left and right is right in the image
            scatter = ax.scatter(
                -xyz[:, 0],  # -X so left appears on left side
                xyz[:, 1],   # Y (forward)
                c=intensity_norm,
                s=0.3,
                cmap='plasma',
                vmin=0, vmax=1
            )

            # Add ego vehicle marker (red triangle pointing up = forward)
            ax.plot(0, 0, 'r^', markersize=10)

            # 1:2 aspect ratio: left-right ±30m, back -30m to front +90m
            ax.set_xlim(-30, 30)
            ax.set_ylim(-30, 90)
            ax.set_aspect('equal')
            ax.set_facecolor('black')
            ax.axis('off')

            fig.savefig(f"{base_path}/lidar_bev_{timestamp:04d}.png",
                        bbox_inches='tight', pad_inches=0, facecolor='black')
            plt.close(fig)

            logger.info(f"Saved LiDAR BEV at {base_path}/lidar_bev_{timestamp:04d}.png")

    def _create_video_from_images(self, image_folder: str, output_video: str, fps: int = 10) -> None:
        """Create MP4 video from image sequence."""
        import cv2
        import glob

        images = sorted(glob.glob(f"{image_folder}/*.png"))
        if not images:
            logger.warning(f"No images found in {image_folder}")
            return

        frame = cv2.imread(images[0])
        h, w = frame.shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter(output_video, fourcc, fps, (w, h))

        for img_path in images:
            video.write(cv2.imread(img_path))

        video.release()
        logger.info(f"Created video: {output_video}")

    def _get_pd_asset(self, sumo_type: str, agent_category: str) -> Tuple[str, AgentType]:
        """Map SUMO vehicle/VRU type to a random PD asset from the appropriate category.

        Args:
            sumo_type: SUMO type ID (e.g., "NDE_URBAN", "bike", "DEFAULT_PEDTYPE")
            agent_category: "vehicle" or "vru" to help disambiguate types

        Returns:
            Tuple of (asset_name, agent_type)
        """
        # Determine category from SUMO type
        pd_category = SUMO_TO_PD_CATEGORY.get(sumo_type, "sedan")  # default to sedan

        # For VRUs, check if it's a bicycle or pedestrian
        if agent_category == "vru":
            if sumo_type in ["bike", "bicycle", "DEFAULT_BIKETYPE"] or "bike" in sumo_type.lower():
                pd_category = "bicycle"
            else:
                pd_category = "pedestrian"

        # Get available assets for this category
        available_assets = PD_ASSETS_BY_CATEGORY.get(pd_category, PD_ASSETS_BY_CATEGORY["sedan"])

        # Random selection using seeded RNG
        selected_asset = self._rng.choice(available_assets)

        # Determine agent type
        if pd_category == "pedestrian":
            agent_type = AgentType.PEDESTRIAN
        elif pd_category == "bicycle":
            agent_type = AgentType.BICYCLE
        else:
            agent_type = AgentType.VEHICLE

        return selected_asset, agent_type

    def _add_vehicle_agent(self, vehicle_key: str, pose: dict) -> bool:
        """Add a new vehicle agent to the world with type-appropriate asset.

        Returns:
            bool: True if agent was added successfully, False otherwise.
        """
        # Get asset based on SUMO type
        sumo_type = pose.get("type", "NDE_URBAN")
        asset_name, agent_type = self._get_pd_asset(sumo_type, "vehicle")

        adjusted_x = pose["center_x"]
        adjusted_y = pose["center_y"]
        vehicle_transform = sdk.Transformation(
            translation=[adjusted_x, adjusted_y, pose["z"]],
            quaternion=sdk.CoordinateSystem("RFU").quaternion_from_rpy(
                roll=0, pitch=0, yaw=-float(pose["sumo_angle"]), degrees=True, order='xyz'
            )
        )

        try:
            if vehicle_key == 'AV':
                logger.info(f"Adding ego vehicle agent (AV) with asset {asset_name}")
                agent = sdk.VehicleAgent(
                    asset=asset_name,
                    pose=self._world.map.get_ground_poses(
                        assets=asset_name,
                        poses=vehicle_transform,
                        mode=sdk.map.GroundPoseMode.GROUND_ALIGNED
                    ),
                    sensor_rig=sdk.SensorRig(
                        sensor_configs=[
                            sdk.examples.sensors.Pinhole,       # Front camera
                            # sdk.examples.sensors.VELODYNE_128,  # 128-beam LiDAR (disabled)
                        ],
                        annotations={sdk.annotations.Depth, sdk.annotations.RGB},  # Removed PointCloud
                    )
                )
            else:
                agent = sdk.VehicleAgent(
                    asset=asset_name,
                    pose=self._world.map.get_ground_poses(
                        assets=asset_name,
                        poses=vehicle_transform,
                        mode=sdk.map.GroundPoseMode.GROUND_ALIGNED
                    ),
                )

            agent.force_steering_angle(0)
            self._world.agents.add(agents=[agent])
            self.agent_id_map[vehicle_key] = agent.id
            self.agent_type_map[vehicle_key] = AgentType.VEHICLE
            self.agent_asset_map[vehicle_key] = asset_name
            logger.info(f"Added vehicle agent {vehicle_key} with asset {asset_name}")
            return True
        except Exception as e:
            logger.error(f"Error adding vehicle agent for key {vehicle_key}: {e}")
            return False

    def _update_vehicle_agent(self, vehicle_key: str, pose: dict) -> bool:
        """Update an existing vehicle agent's pose.

        Returns:
            bool: True if agent was updated successfully, False otherwise.
        """
        generated_agent_id = self.agent_id_map.get(vehicle_key)
        # Use the stored asset name from creation (for correct ground alignment calculation)
        asset_name = self.agent_asset_map.get(vehicle_key)

        if generated_agent_id is None or asset_name is None:
            return False

        adjusted_x = pose["center_x"]
        adjusted_y = pose["center_y"]
        vehicle_transform = sdk.Transformation(
            translation=[adjusted_x, adjusted_y, pose["z"]],
            quaternion=sdk.CoordinateSystem("RFU").quaternion_from_rpy(
                roll=0, pitch=0, yaw=-float(pose["sumo_angle"]), degrees=True, order='xyz'
            )
        )

        try:
            vehicle_agent = self._world.agents.get_by_id(agent_id=generated_agent_id)
            vehicle_agent.update_pose(
                self._world.map.get_ground_poses(
                    assets=asset_name,  # Use stored asset for correct ground alignment
                    poses=vehicle_transform,
                    mode=sdk.map.GroundPoseMode.GROUND_ALIGNED
                )
            )
            vehicle_agent.force_steering_angle(0)
            return True
        except Exception as e:
            logger.error(f"Error updating vehicle agent for key {vehicle_key}: {e}")
            return False

    def _remove_agent(self, agent_key: str) -> bool:
        """Remove any type of agent from the world.

        Returns:
            bool: True if agent was removed successfully, False otherwise.
        """
        generated_agent_id = self.agent_id_map.get(agent_key)
        if generated_agent_id is None:
            return False

        try:
            agent = self._world.agents.get_by_id(agent_id=generated_agent_id)
            self._world.agents.remove(agents=[agent])
            del self.agent_id_map[agent_key]
            if agent_key in self.agent_type_map:
                del self.agent_type_map[agent_key]
            if agent_key in self.agent_asset_map:
                del self.agent_asset_map[agent_key]
            logger.info(f"Removed agent {agent_key}")
            return True
        except Exception as e:
            logger.error(f"Error removing agent for key {agent_key}: {e}")
            return False

    def _add_bicycle_agent(self, agent_key: str, pose: dict) -> bool:
        """Add a bicycle agent to the world using VehicleAgent with bicycle asset.

        Returns:
            bool: True if agent was added successfully, False otherwise.
        """
        sumo_type = pose.get("type", "bike")
        asset_name, _ = self._get_pd_asset(sumo_type, "vru")

        adjusted_x = pose["center_x"]
        adjusted_y = pose["center_y"]
        bicycle_transform = sdk.Transformation(
            translation=[adjusted_x, adjusted_y, pose["z"]],
            quaternion=sdk.CoordinateSystem("RFU").quaternion_from_rpy(
                roll=0, pitch=0, yaw=-float(pose["sumo_angle"]), degrees=True, order='xyz'
            )
        )

        try:
            agent = sdk.VehicleAgent(
                asset=asset_name,
                pose=self._world.map.get_ground_poses(
                    assets=asset_name,
                    poses=bicycle_transform,
                    mode=sdk.map.GroundPoseMode.GROUND_ALIGNED
                ),
            )
            self._world.agents.add(agents=[agent])
            self.agent_id_map[agent_key] = agent.id
            self.agent_type_map[agent_key] = AgentType.BICYCLE
            self.agent_asset_map[agent_key] = asset_name
            logger.info(f"Added bicycle agent {agent_key} with asset {asset_name}")
            return True
        except Exception as e:
            logger.error(f"Error adding bicycle agent for key {agent_key}: {e}")
            return False

    def _update_bicycle_agent(self, agent_key: str, pose: dict) -> bool:
        """Update an existing bicycle agent's pose.

        Returns:
            bool: True if agent was updated successfully, False otherwise.
        """
        generated_agent_id = self.agent_id_map.get(agent_key)
        # Use the stored asset name from creation (for correct ground alignment calculation)
        asset_name = self.agent_asset_map.get(agent_key)

        if generated_agent_id is None or asset_name is None:
            return False

        adjusted_x = pose["center_x"]
        adjusted_y = pose["center_y"]
        bicycle_transform = sdk.Transformation(
            translation=[adjusted_x, adjusted_y, pose["z"]],
            quaternion=sdk.CoordinateSystem("RFU").quaternion_from_rpy(
                roll=0, pitch=0, yaw=-float(pose["sumo_angle"]), degrees=True, order='xyz'
            )
        )

        try:
            bicycle_agent = self._world.agents.get_by_id(agent_id=generated_agent_id)
            bicycle_agent.update_pose(
                self._world.map.get_ground_poses(
                    assets=asset_name,  # Use stored asset for correct ground alignment
                    poses=bicycle_transform,
                    mode=sdk.map.GroundPoseMode.GROUND_ALIGNED
                )
            )
            return True
        except Exception as e:
            logger.error(f"Error updating bicycle agent for key {agent_key}: {e}")
            return False

    def _add_pedestrian_agent(self, agent_key: str, pose: dict) -> bool:
        """Add a pedestrian agent to the world using HumanAgent with walking animation.

        Returns:
            bool: True if agent was added successfully, False otherwise.
        """
        sumo_type = pose.get("type", "pedestrian")
        asset_name, _ = self._get_pd_asset(sumo_type, "vru")

        adjusted_x = pose["center_x"]
        adjusted_y = pose["center_y"]
        pedestrian_transform = sdk.Transformation(
            translation=[adjusted_x, adjusted_y, pose["z"]],
            quaternion=sdk.CoordinateSystem("RFU").quaternion_from_rpy(
                roll=0, pitch=0, yaw=-float(pose["sumo_angle"]), degrees=True, order='xyz'
            )
        )

        try:
            # Use HumanAgent for pedestrians - supports physics and animations
            agent = sdk.HumanAgent(
                asset=asset_name,
                pose=self._world.map.get_ground_poses(
                    assets=asset_name,
                    poses=pedestrian_transform,
                    mode=sdk.map.GroundPoseMode.GROUND_ALIGNED
                ),
            )
            self._world.agents.add(agents=[agent])

            # Set walking animation state after agent is added to world
            try:
                agent.set_animation_states(skip_transition=False, animation_state="walk")
            except Exception as anim_e:
                logger.warning(f"Could not set walking animation for {agent_key}: {anim_e}")

            self.agent_id_map[agent_key] = agent.id
            self.agent_type_map[agent_key] = AgentType.PEDESTRIAN
            self.agent_asset_map[agent_key] = asset_name
            self.agent_last_pose[agent_key] = (adjusted_x, adjusted_y)  # Store initial position
            logger.info(f"Added pedestrian agent {agent_key} with asset {asset_name} (HumanAgent with walk animation)")
            return True
        except Exception as e:
            logger.error(f"Error adding pedestrian agent for key {agent_key}: {e}")
            return False

    def _update_pedestrian_agent(self, agent_key: str, pose: dict, time_delta: float = 0.1) -> bool:
        """Update an existing pedestrian agent's pose with speed-based animation.

        Calculates movement speed from position delta and adjusts animation state:
        - speed < 0.3 m/s: idle (standing still)
        - speed < 1.8 m/s: walk
        - speed >= 1.8 m/s: run

        Args:
            agent_key: The unique identifier for this agent
            pose: Dict containing center_x, center_y, z, sumo_angle
            time_delta: Time between updates in seconds (default 0.1s = 10Hz)

        Returns:
            bool: True if agent was updated successfully, False otherwise.
        """
        generated_agent_id = self.agent_id_map.get(agent_key)
        asset_name = self.agent_asset_map.get(agent_key)

        if generated_agent_id is None or asset_name is None:
            return False

        adjusted_x = pose["center_x"]
        adjusted_y = pose["center_y"]

        # Calculate speed from position delta
        last_pose = self.agent_last_pose.get(agent_key)
        if last_pose is not None:
            dx = adjusted_x - last_pose[0]
            dy = adjusted_y - last_pose[1]
            distance = (dx**2 + dy**2) ** 0.5
            speed = distance / time_delta  # m/s
        else:
            speed = 0.0

        # Update stored position
        self.agent_last_pose[agent_key] = (adjusted_x, adjusted_y)

        pedestrian_transform = sdk.Transformation(
            translation=[adjusted_x, adjusted_y, pose["z"]],
            quaternion=sdk.CoordinateSystem("RFU").quaternion_from_rpy(
                roll=0, pitch=0, yaw=-float(pose["sumo_angle"]), degrees=True, order='xyz'
            )
        )

        try:
            pedestrian_agent = self._world.agents.get_by_id(agent_id=generated_agent_id)

            # Update pose
            pedestrian_agent.update_pose(
                self._world.map.get_ground_poses(
                    assets=asset_name,
                    poses=pedestrian_transform,
                    mode=sdk.map.GroundPoseMode.GROUND_ALIGNED
                )
            )

            # Adjust animation based on speed
            try:
                if speed < 0.3:
                    # Standing still or very slow
                    pedestrian_agent.set_animation_states(skip_transition=True, animation_state="idle")
                elif speed < 1.8:
                    # Normal walking speed (typical: 1.2-1.5 m/s)
                    pedestrian_agent.set_animation_states(skip_transition=True, animation_state="walk")
                else:
                    # Fast movement - running
                    pedestrian_agent.set_animation_states(skip_transition=True, animation_state="run")
            except Exception as anim_e:
                pass  # Silently ignore animation errors to avoid log spam

            return True
        except Exception as e:
            logger.error(f"Error updating pedestrian agent for key {agent_key}: {e}")
            return False

    def _is_bicycle_type(self, vru_type: str) -> bool:
        """Check if VRU type indicates a bicycle."""
        return vru_type in ["bike", "bicycle", "DEFAULT_BIKETYPE"] or "bike" in vru_type.lower()

    def _build_state(self, input_state: dict, is_first_timestep: bool = False):
        """
        Update the PD world state based on the current simulation state.

        This method handles dynamic addition, update, and removal of all agent types:
        - Vehicles (cars, trucks, buses, etc.)
        - Bicycles (VRUs with bike type)
        - Pedestrians (VRUs with person type)
        """
        agent_state = input_state.get("agent_details", {})

        # Get current vehicles and VRUs
        current_vehicles = agent_state.get("vehicle", {})
        current_vrus = agent_state.get("vru", {})
        # Filter VRUs to only keep bicycles (bike type)
        # current_vrus = {k: v for k, v in current_vrus.items() if self._is_bicycle_type(v.get("type", ""))}

        # Combine all agent keys for tracking
        current_all_keys = set(current_vehicles.keys()) | set(current_vrus.keys())
        existing_keys = set(self.agent_id_map.keys())

        # Find agents to add, remove, update
        agents_to_add = current_all_keys - existing_keys
        agents_to_remove = existing_keys - current_all_keys
        agents_to_update = current_all_keys & existing_keys

        # Log state changes
        if agents_to_add:
            logger.info(f"Adding {len(agents_to_add)} new agents")
        if agents_to_remove:
            logger.info(f"Removing {len(agents_to_remove)} agents")
        if agents_to_update:
            logger.debug(f"Updating {len(agents_to_update)} agents")

        # Remove agents that left the area
        for agent_key in agents_to_remove:
            self._remove_agent(agent_key)

        # Add new agents
        for agent_key in agents_to_add:
            if agent_key in current_vehicles:
                pose = current_vehicles[agent_key]
                self._add_vehicle_agent(agent_key, pose)
            elif agent_key in current_vrus:
                pose = current_vrus[agent_key]
                vru_type = pose.get("type", "")

                # Determine if VRU is a bicycle or pedestrian based on type
                if self._is_bicycle_type(vru_type):
                    self._add_bicycle_agent(agent_key, pose)
                else:
                    self._add_pedestrian_agent(agent_key, pose)

        # Update existing agents
        for agent_key in agents_to_update:
            if agent_key in current_vehicles:
                pose = current_vehicles[agent_key]
                self._update_vehicle_agent(agent_key, pose)
            elif agent_key in current_vrus:
                pose = current_vrus[agent_key]
                agent_type = self.agent_type_map.get(agent_key)

                if agent_type == AgentType.BICYCLE:
                    self._update_bicycle_agent(agent_key, pose)
                elif agent_type == AgentType.PEDESTRIAN:
                    self._update_pedestrian_agent(agent_key, pose)

    def run_simulation(self, config_file: str = "examples/scenarios/Mcity_safety_assessment.yaml", auto_run: bool = False, initialize_timeout: int = 3600, tick_timeout: int = 3600, enable_viz: bool = False, viz_port: int = 8050, viz_update_freq: int = 2) -> dict:
        """Run a full simulation loop via the HTTP API and return the simulation result JSON.

        This mirrors the flow in scripts/run_experiments.py:
        - POST /start_simulation (with optional viz params)
        - poll /simulation_status until "wait_for_tick"
        - loop: POST /simulation_tick and poll until 'ticked' or 'finished'
        - fetch /simulation_result when finished
        """
        base_url = "http://localhost:8000"

        # Load World API
        if self._world is None:
            self._world = sdk.World(
                data_lab_instance=self._instance_name,
                world_config=sdk.WorldConfig(lighting=self._lighting),
                unique_scene_name=f"simulation_{uuid4()}",
                location=self._location,
                seed=self._seed,
            )
            logger.info("Initialized Parallel Domain World API client")
        
        # Start simulation
        start_response = requests.post(
            f"{base_url}/start_simulation",
            json={
                "config_file": config_file,
                "auto_run": auto_run
            },
            params={
                "enable_viz": enable_viz,
                "viz_port": viz_port,
                "viz_update_freq": viz_update_freq
            }
        )
        response_data = start_response.json()
        self.simulation_id = response_data["simulation_id"]
        simulation_id = response_data["simulation_id"]
        
        # Print visualization URL if enabled
        if enable_viz and "visualization_url" in response_data:
            print(f"🎨 Visualization available at: {response_data['visualization_url']}")
            print(f"   Open this URL in your browser to see real-time visualization")

        start_time = time.time()
        while True:
            # Get simulation status
            try:
                status_response = requests.get(f"{base_url}/simulation_status/{simulation_id}")
                # print(f"Simulation status: {status_response.json()}")
                # Break if simulation is waiting for tick
                if status_response.json()["status"] == "wait_for_tick":
                    break
                if time.time() - start_time > initialize_timeout:  # 10 seconds timeout
                    print("Simulation initialization timeout, stopping...")
                    requests.post(f"{base_url}/simulation_control/{simulation_id}", json={"command": "stop"})
                    raise TimeoutError("Simulation initialization timeout")
            except Exception as e:
                print(f"Simulation status not ready: {e}")
                time.sleep(0.05)

        # Main tick loop
        i = 0
        with self._world:
            while True:
                # Tick once
                self.tick(simulation_id)
                # Wait until ticked or finished
                start_time = time.time()
                while True:
                    status = self.check_simulation_status(simulation_id)
                    s = status.get("status")
                    # Print status for debugging
                    # logger.info(f"Simulation status: {s}")
                    if s in ("ticked", "finished"):
                        break
                    if time.time() - start_time > tick_timeout:
                        logger.error("Simulation tick timeout, stopping")
                        self.control_simulation(simulation_id, "stop")
                        return {"error": "Simulation timeout"}
                    time.sleep(0.02)

                if s == "finished":
                    return

                # Optionally retrieve state per tick with optional filtering
                tick_state = self.get_state(simulation_id, center_id=self._filter_center_id, radius=self._filter_radius)
                # self.sim_states.append(tick_state)  # store last state

                # Here you could update the parallel domain world based on tick_state if needed
                # Call the "external simulator" with the control input and build the world state
                logger.info(f"Building world state for timestep {i}")
                self._build_state(tick_state, is_first_timestep=(i==0))
                capture_this_frame = i % self._capture_rate == 0 and i != 0
                logger.info(f"Ticking and rendering world for timestep {i}, capture frame: {capture_this_frame}")
                self._world.tick_and_render(
                    capture=capture_this_frame,
                    visualize=self._visualize,
                    update_world=True
                )
                # Save RGB images when capturing frames
                if capture_this_frame:
                    self._retrieve_rgb(i)
                    # self._retrieve_lidar_bev(i)  # LiDAR BEV disabled
                i += 1

        # Create videos from captured images
        self._create_video_from_images("./output_images/camera_0/", "./rgb_video.mp4")
        # self._create_video_from_images("./output_lidar_bev/", "./lidar_bev_video.mp4")  # LiDAR disabled

        # Get final results
        result = self.get_result(simulation_id)
        return result

    def get_simulation_id(self) -> Optional[str]:
        """Return the last-started simulation id stored in memory (or None)."""
        return self.simulation_id

def main():
    # Simple CLI usage when run directly
    import argparse

    parser = argparse.ArgumentParser(description="TeraSim client helper")
    parser.add_argument("--config", help="Path to simulation config file", default="examples/scenarios/Mcity_safety_assessment.yaml")
    parser.add_argument("--auto-run", action="store_true", dest="auto_run")
    parser.add_argument("--base-url", default="http://localhost:8000")
    args = parser.parse_args()

    # Set up the context and loggers
    context = sdk.setup()
    sdk.setup_loggers()
    context.enable_distributed_rendering = True

    # Setup key parameters of the simulation
    lighting = "day_clear_01"
    location = "Mcity"
    seed = 2025

    client = TeraSimClient(
        base_url=args.base_url,
        location=location,
        lighting=lighting,
        seed=seed,
        simulation_length_s=6.0,
        framerate_fps=10.0, 
        instance_name="llwoubqj")
    result = client.run_simulation(config_file=args.config, tick_timeout= 4800, auto_run=args.auto_run, viz_update_freq=2, enable_viz=False)
    print(f"Simulation result: {result}")


if __name__ == "__main__":
    main()
