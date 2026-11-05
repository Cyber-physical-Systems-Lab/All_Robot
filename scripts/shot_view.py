#!/usr/bin/env python3
"""Camera framing helpers for scripts/screenshot_ros.sh (pure Python, no ROS import).

  python3 scripts/shot_view.py rviz <robot> <out.rviz>   # RViz2 config: robot model + orbit view
  python3 scripts/shot_view.py gz-pose <robot>           # ignition.msgs.GUICamera request text
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import CAM_AZIMUTH_DEG, CAM_ELEVATION_DEG, camera_view, load_registry  # noqa: E402


def rviz_config(robot: str, out: Path, width: int = 960, height: int = 720) -> None:
    cfg = load_registry()["robots"][robot]
    eye, target = camera_view(cfg)
    cfg_rviz = {
        "Panels": [],
        "Visualization Manager": {
            "Class": "", "Enabled": True,
            "Displays": [
                {"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True,
                 "Cell Size": 0.25, "Plane Cell Count": 16},
                {"Class": "rviz_default_plugins/RobotModel", "Name": "RobotModel", "Enabled": True,
                 "Description Source": "Topic",
                 "Description Topic": {"Value": "/robot_description", "Depth": 5,
                                       "Durability Policy": "Transient Local",
                                       "History Policy": "Keep Last", "Reliability Policy": "Reliable"},
                 "Visual Enabled": True, "Collision Enabled": False},
            ],
            "Global Options": {"Fixed Frame": "base_link", "Background Color": "48; 48; 48"},
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"}],
            "Views": {"Current": {
                "Class": "rviz_default_plugins/Orbit", "Name": "Current View",
                "Distance": math.dist(eye, target),
                "Focal Point": {"X": target[0], "Y": target[1], "Z": target[2]},
                "Yaw": math.radians(CAM_AZIMUTH_DEG), "Pitch": math.radians(CAM_ELEVATION_DEG)}}},
        "Window Geometry": {"Width": width, "Height": height, "X": 0, "Y": 0,
                            "Hide Left Dock": True, "Hide Right Dock": True},
    }
    out.write_text(yaml.safe_dump(cfg_rviz, sort_keys=False))


def gz_pose(robot: str) -> str:
    """Gazebo GUI camera (x forward): yaw toward the target, positive pitch looks down."""
    eye, _ = camera_view(load_registry()["robots"][robot])
    yaw = math.radians(CAM_AZIMUTH_DEG + 180.0)
    pitch = math.radians(CAM_ELEVATION_DEG)
    cp, sp, cy, sy = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    qw, qx, qy, qz = cp * cy, -sp * sy, sp * cy, cp * sy
    return (f"pose: {{position: {{x: {eye[0]:.4f}, y: {eye[1]:.4f}, z: {eye[2]:.4f}}}, "
            f"orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}}}")


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "rviz":
        rviz_config(sys.argv[2], Path(sys.argv[3]))
    elif len(sys.argv) == 3 and sys.argv[1] == "gz-pose":
        print(gz_pose(sys.argv[2]))
    else:
        sys.exit(__doc__)
