#!/usr/bin/env python3
"""Visualize conflict trajectories from a JSONL file on a SUMO network map.

Each JSONL line contains a conflict event with adversarial and conflict vehicle
trajectories as arrays of waypoints [x, y, heading, velocity, time].

Usage:
    python visualize_conflict.py config.yaml --jsonl /path/to/conflicts.jsonl --net /path/to/map.net.xml --output_dir /path/to/output/ --incident 0
"""

import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from pathlib import Path
import sys
import yaml
import argparse
import time as _time

from terasim_vis import Net, Trajectory


def load_config(config_path):
    """Load configuration from YAML file."""
    config_path = Path(config_path)

    if not config_path.exists():
        print(f"Error: Configuration file {config_path} not found.")
        sys.exit(1)

    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        config.setdefault("animate", False)
        config.setdefault("dpi", 150)
        config.setdefault("figsize", "12,8")

        return config

    except Exception as e:
        print(f"Error loading configuration file: {e}")
        sys.exit(1)


def load_conflict_event(jsonl_path, incident):
    """Load a single conflict event from a JSONL file by incident number (0-indexed line).

    Returns a dict with keys:
        timestamp, adversarial_vehicle, conflict_vehicle,
        adversarial_trajectory, conflict_trajectory
    """
    with open(jsonl_path, "r") as f:
        for line_num, line in enumerate(f):
            if line_num == incident:
                line = line.strip()
                if not line:
                    print(f"Error: incident {incident} is an empty line.")
                    sys.exit(1)
                return json.loads(line)

    print(f"Error: incident {incident} out of range (file has {line_num + 1} lines).")
    sys.exit(1)


def build_trajectory(vehicle_id, waypoints, color="red"):
    """Build a Trajectory object from raw waypoints.

    Args:
        vehicle_id: Vehicle identifier string.
        waypoints: List of [x, y, heading, velocity, time] arrays.
        color: Color for the trajectory.

    Returns:
        A Trajectory object ready for plotting.
    """
    traj = Trajectory(id=vehicle_id, type="car")
    for wp in waypoints:
        x, y, heading, velocity, time = wp
        traj._append_point(
            time=time,
            x=x,
            y=y,
            speed=velocity,
            angle=heading,
            color=color,
        )
    traj.length = 4.8
    traj.width = 2.0
    return traj


class ConflictVisualizer:
    """Visualize conflict trajectories on a SUMO network map."""

    def __init__(self, net, config):
        self.net = net
        self.config = config
        self.fig = None
        self.ax = None

    def setup_plot(self):
        """Set up the matplotlib figure and plot the network."""
        figsize = tuple(map(float, self.config["figsize"].split(",")))
        self.fig, self.ax = plt.subplots(figsize=figsize)

        map_style = self.config.get("map_style", {})
        colors = map_style.get("colors", {})
        lane_color = colors.get("lane_color", "white")

        self.net.plot(
            self.ax,
            style=map_style.get("style", "EUR"),
            zoom_to_extents=map_style.get("zoom_to_extents", True),
            plot_stop_lines=map_style.get("plot_stop_lines", False),
            lane_kwargs=dict(color=lane_color),
            junction_kwargs=dict(color=lane_color),
            tl_phases=map_style.get("tl_phases", None),
        )

        self.ax.set_aspect("equal")

        if not map_style.get("zoom_to_extents", True):
            self._set_custom_view(map_style.get("view", {}))

        if map_style.get("show_title", False):
            self.ax.set_title(
                map_style.get("title", "Conflict Trajectory Visualization")
            )

        if not map_style.get("show_labels", False):
            self.ax.set_xticks([])
            self.ax.set_yticks([])

        self.ax.margins(0)
        self.ax.set_aspect("equal", adjustable="box")
        plt.subplots_adjust(left=0, right=1, top=1, bottom=0)

        return self.fig, self.ax

    def _set_custom_view(self, view_config, center_xy=None):
        """Set custom view boundaries."""
        width = view_config.get("width", 200)
        height = view_config.get("height", 200)

        if center_xy is not None:
            center_x, center_y = center_xy
        else:
            center_x = view_config.get("center_x", 0)
            center_y = view_config.get("center_y", 0)

        half_w = width / 2
        half_h = height / 2
        self.ax.set_xlim(center_x - half_w, center_x + half_w)
        self.ax.set_ylim(center_y - half_h, center_y + half_h)

    def _center_on_trajectories(self, trajectories, view_config):
        """Center the view on the midpoint of given trajectories."""
        all_x, all_y = [], []
        for traj in trajectories:
            all_x.extend(traj.x)
            all_y.extend(traj.y)

        if all_x:
            cx = (min(all_x) + max(all_x)) / 2
            cy = (min(all_y) + max(all_y)) / 2
            self._set_custom_view(view_config, center_xy=(cx, cy))

    def plot_static_event(self, adv_traj, conflict_traj):
        """Plot a single conflict event as static trajectory lines."""
        adv_traj.assign_colors_constant("red")
        conflict_traj.assign_colors_constant("blue")

        adv_traj.plot(self.ax, linewidth=2.0, alpha=0.8)
        conflict_traj.plot(self.ax, linewidth=2.0, alpha=0.8)

        # Mark start positions with circles
        if adv_traj.x:
            self.ax.plot(
                adv_traj.x[0], adv_traj.y[0], "o",
                color="red", markersize=8, markeredgecolor="black",
                zorder=300, label=f"Adv: {adv_traj.id}",
            )
        if conflict_traj.x:
            self.ax.plot(
                conflict_traj.x[0], conflict_traj.y[0], "o",
                color="blue", markersize=8, markeredgecolor="black",
                zorder=300, label=f"Conflict: {conflict_traj.id}",
            )

        # Mark end positions with squares
        if adv_traj.x:
            self.ax.plot(
                adv_traj.x[-1], adv_traj.y[-1], "s",
                color="red", markersize=8, markeredgecolor="black", zorder=300,
            )
        if conflict_traj.x:
            self.ax.plot(
                conflict_traj.x[-1], conflict_traj.y[-1], "s",
                color="blue", markersize=8, markeredgecolor="black", zorder=300,
            )

        self.ax.legend(loc="upper right", fontsize=8)

        # Auto-center if configured
        map_style = self.config.get("map_style", {})
        view_config = map_style.get("view", {})
        if view_config.get("center_on_conflict", True) and not map_style.get(
            "zoom_to_extents", True
        ):
            self._center_on_trajectories([adv_traj, conflict_traj], view_config)

    def create_animation(self, adv_traj, conflict_traj):
        """Create an animated visualization of the conflict event."""
        adv_traj.assign_colors_constant("red")
        adv_traj.point_plot_kwargs["color"] = "red"
        adv_traj.point_plot_kwargs["markeredgecolor"] = "black"
        adv_traj.point_plot_kwargs["zorder"] = 300
        adv_traj.point_plot_kwargs["ms"] = 2

        conflict_traj.assign_colors_constant("blue")
        conflict_traj.point_plot_kwargs["color"] = "blue"
        conflict_traj.point_plot_kwargs["markeredgecolor"] = "black"
        conflict_traj.point_plot_kwargs["zorder"] = 290
        conflict_traj.point_plot_kwargs["ms"] = 2

        # Determine shared time range
        all_times = sorted(set(adv_traj.time + conflict_traj.time))
        timestep_range = np.array(all_times)

        map_style = self.config.get("map_style", {})
        view_config = map_style.get("view", {})
        follow = view_config.get("center_on_conflict", True) and not map_style.get(
            "zoom_to_extents", True
        )

        # Build lookup dicts for fast per-frame access
        adv_lookup = {t: i for i, t in enumerate(adv_traj.time)}
        conflict_lookup = {t: i for i, t in enumerate(conflict_traj.time)}

        # Keep track of artists to remove between frames
        self._frame_artists = []

        def animate(frame):
            # Remove previous frame artists
            for artist in self._frame_artists:
                artist.remove()
            self._frame_artists.clear()

            artists = []

            # Draw adversarial vehicle
            if frame in adv_lookup:
                idx = adv_lookup[frame]
                a = _draw_vehicle_box(
                    self.ax, adv_traj.x[idx], adv_traj.y[idx],
                    adv_traj.angle[idx], adv_traj.length, adv_traj.width,
                    facecolor="red", edgecolor="black", zorder=300,
                )
                artists.append(a)

            # Draw conflict vehicle
            if frame in conflict_lookup:
                idx = conflict_lookup[frame]
                a = _draw_vehicle_box(
                    self.ax, conflict_traj.x[idx], conflict_traj.y[idx],
                    conflict_traj.angle[idx], conflict_traj.length, conflict_traj.width,
                    facecolor="blue", edgecolor="black", zorder=290,
                )
                artists.append(a)

            self._frame_artists = artists

            # Follow the midpoint of both vehicles
            if follow:
                xs, ys = [], []
                if frame in adv_lookup:
                    i = adv_lookup[frame]
                    xs.append(adv_traj.x[i])
                    ys.append(adv_traj.y[i])
                if frame in conflict_lookup:
                    i = conflict_lookup[frame]
                    xs.append(conflict_traj.x[i])
                    ys.append(conflict_traj.y[i])
                if xs:
                    cx = sum(xs) / len(xs)
                    cy = sum(ys) / len(ys)
                    self._set_custom_view(view_config, center_xy=(cx, cy))

            return artists

        # Compute timestep for interval
        if len(timestep_range) > 1:
            dt = timestep_range[1] - timestep_range[0]
        else:
            dt = 0.5

        anim = animation.FuncAnimation(
            self.fig,
            animate,
            frames=timestep_range,
            repeat=False,
            interval=int(1000 * dt),
            blit=False,
        )
        anim.time_range = timestep_range
        return anim


def _draw_vehicle_box(ax, x, y, angle_deg, length, width, **kwargs):
    """Draw a rotated rectangle representing a vehicle.

    Args:
        ax: Matplotlib axes.
        x, y: Center position.
        angle_deg: Heading in degrees (SUMO convention: 0=north, clockwise).
        length, width: Vehicle dimensions.
        **kwargs: Passed to matplotlib Polygon patch.

    Returns:
        The Polygon artist added to the axes.
    """
    # Half-dimensions
    hl = length / 2
    hw = width / 2

    # Rectangle corners relative to center (oriented along heading)
    corners = np.array([
        [-hl, -hw],
        [hl, -hw],
        [hl, hw],
        [-hl, hw],
    ])

    # Rotation matrix (angle_deg is heading: 0=north, CW positive)
    # In a standard x-right y-up system, heading 0 means pointing up (+y)
    # So we rotate by -(angle - 90) = (90 - angle)
    rot_angle = np.radians(90 - angle_deg)
    cos_a, sin_a = np.cos(rot_angle), np.sin(rot_angle)
    rotation = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    rotated = corners @ rotation.T
    rotated[:, 0] += x
    rotated[:, 1] += y

    poly = plt.Polygon(rotated, closed=True, **kwargs)
    ax.add_patch(poly)
    return poly


    


def main(args):
    """Main function."""
    config = load_config(args.config)

    jsonl_path = Path(args.jsonl)
    net_path = Path(args.net)
    output_dir = Path(args.output_dir)
    incident = args.incident

    # Load event first so we can use vehicle position for spatial filtering
    print(f"Loading conflict event (incident {incident}) from: {jsonl_path}")
    event = load_conflict_event(jsonl_path, incident)

    adv_id = event["adversarial_vehicle"]
    conflict_id = event["conflict_vehicle"]
    timestamp = event["timestamp"]

    print(f"Incident {incident}: timestamp={timestamp}, "
          f"adv={adv_id}, conflict={conflict_id}")

    # Use the conflict vehicle's initial position as center for map filtering
    conflict_wp0 = event["conflict_trajectory"][0]
    center = (conflict_wp0[0], conflict_wp0[1])
    radius = config.get("net_radius", None)

    print(f"Loading network from: {net_path}")
    if radius is not None:
        print(f"  Spatial filter: center=({center[0]:.1f}, {center[1]:.1f}), radius={radius}m")
    t0 = _time.time()
    net = Net(str(net_path), center=center, radius=radius)
    print(f"Network loaded in {_time.time() - t0:.2f}s")

    output_dir.mkdir(parents=True, exist_ok=True)

    adv_traj = build_trajectory(adv_id, event["adversarial_trajectory"], color="red")
    conflict_traj = build_trajectory(
        conflict_id, event["conflict_trajectory"], color="blue"
    )

    visualizer = ConflictVisualizer(net, config)
    visualizer.setup_plot()

    if config["animate"]:
        print("Creating animation...")
        anim = visualizer.create_animation(adv_traj, conflict_traj)

        video_name = config.get("video_name", f"conflict_{incident}")
        save_path = output_dir / f"{video_name}.mp4"
        print(f"Saving animation to: {save_path}")

        Writer = animation.writers["ffmpeg"]
        writer = Writer(
            fps=config.get("fps", 10),
            metadata=dict(artist="TeraSim-Vis"),
            bitrate=3600,
        )
        print(f"Saving {len(anim.time_range)} frames at {config.get('fps', 10)} fps...")
        anim.save(str(save_path), writer=writer, dpi=config["dpi"])
        print("Animation saved!")
    else:
        print("Creating static visualization...")
        visualizer.plot_static_event(adv_traj, conflict_traj)

        save_path = output_dir / f"conflict_{incident}.png"
        print(f"Saving plot to: {save_path}")
        plt.savefig(
            str(save_path), dpi=config["dpi"], bbox_inches="tight", pad_inches=0
        )

    plt.close()
    print("\nVisualization complete!")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Visualize conflict trajectories from JSONL data on a SUMO map.")
    parser.add_argument("--config", type=str, 
        default="configs/visulation/conflict_example.yaml", help="Path to the configuration YAML file.")
    parser.add_argument("--jsonl", type=str, 
        default="/home/jiawei/data/terasim_v2/ann_arbor_conflict_av/conflict_info_withTraj.jsonl", help="Path to the JSONL conflict trajectory file.")
    parser.add_argument("--net", type=str, 
        default="/home/jiawei/data/terasim_v2/ann_arbor_conflict_av/map_fixed.net.xml", help="Path to the SUMO network (.net.xml) file.")
    parser.add_argument("--output_dir", type=str, 
        default="/home/jiawei/data/terasim_v2/ann_arbor_conflict_av/terasim_vis", help="Output directory for images/videos.")
    parser.add_argument("--incident", type=int, 
        default=0, help="Incident number (0-indexed line in the JSONL file).")
    args = parser.parse_args()
    
    main(args)
