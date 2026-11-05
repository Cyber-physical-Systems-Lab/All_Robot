#!/usr/bin/env python3
"""Camera framing helpers for scripts/screenshot_ros.sh (pure Python, no ROS import).

  python3 scripts/shot_view.py rviz <robot> <out.rviz>   # RViz2 config: robot model + orbit view
  python3 scripts/shot_view.py gz-pose <robot>           # ignition.msgs.GUICamera request text
  python3 scripts/shot_view.py gz-gui <robot> <out.config>  # Gazebo GUI: 3D view only, camera preset
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import CAM_AZIMUTH_DEG, CAM_ELEVATION_DEG, camera_view, load_registry  # noqa: E402


def rviz_config(robot: str, out: Path, width: int = 1010, height: int = 829) -> None:
    """RViz window sized so the 3D render panel is ~960x720 (same as the other simulators)."""
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


GZ_APP_BAR_PX = 48   # height of Gazebo's title bar, cropped off by screenshot_ros.sh


def gz_gui_config(robot: str, out: Path, width: int = 960, height: int = 720, fovy_deg: float = 45.0) -> None:
    """Gazebo Fortress GUI config showing only the 3D scene, camera already at the shared view.

    The window is GZ_APP_BAR_PX taller than the image; the horizontal FOV matches MuJoCo's 45 deg fovy
    (MinimalScene's <horizontal_fov> is in degrees).
    """
    eye, _ = camera_view(load_registry()["robots"][robot])
    hfov = 2 * math.atan(math.tan(math.radians(fovy_deg) / 2) * width / height)
    yaw, pitch = math.radians(CAM_AZIMUTH_DEG + 180.0), math.radians(CAM_ELEVATION_DEG)
    hidden = ('<ignition-gui><property key="state" type="string">floating</property>'
              '<property key="width" type="double">5</property><property key="height" type="double">5</property>'
              '<property key="showTitleBar" type="bool">false</property></ignition-gui>')
    out.write_text(f"""<?xml version="1.0"?>
<window>
  <width>{width}</width>
  <height>{height + GZ_APP_BAR_PX}</height>
  <menus><drawer visible="false"/><plugins visible="false"/></menus>
  <dialog_on_exit>false</dialog_on_exit>
</window>
<plugin filename="MinimalScene" name="3D View">
  <ignition-gui>
    <title>3D View</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="string" key="state">docked</property>
  </ignition-gui>
  <engine>ogre2</engine>
  <scene>scene</scene>
  <ambient_light>0.4 0.4 0.4</ambient_light>
  <background_color>0.8 0.8 0.8</background_color>
  <camera_pose>{eye[0]:.4f} {eye[1]:.4f} {eye[2]:.4f} 0 {pitch:.5f} {yaw:.5f}</camera_pose>
  <horizontal_fov>{math.degrees(hfov):.3f}</horizontal_fov>
</plugin>
<plugin filename="GzSceneManager" name="Scene Manager">{hidden}</plugin>
<plugin filename="InteractiveViewControl" name="Interactive view control">{hidden}</plugin>
<plugin filename="CameraTracking" name="Camera Tracking">{hidden}</plugin>
""")


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "rviz":
        rviz_config(sys.argv[2], Path(sys.argv[3]))
    elif len(sys.argv) >= 4 and sys.argv[1] == "gz-gui":
        gz_gui_config(sys.argv[2], Path(sys.argv[3]))
    elif len(sys.argv) == 3 and sys.argv[1] == "gz-pose":
        print(gz_pose(sys.argv[2]))
    else:
        sys.exit(__doc__)
