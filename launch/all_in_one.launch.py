"""All lab robots side by side in RViz2, each in its own namespace / TF prefix.

  ros2 launch cps_robot_description all_in_one.launch.py
  ros2 launch cps_robot_description all_in_one.launch.py robots:=go2,nova5_2f85 spacing:=1.5
"""
import os
import sys
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cps_launch_utils as u  # noqa: E402

DEFAULT = "go2,myagv,mycobot_280_pi,mycomb,nova5_2f85"


def _rviz_config(names):
    displays = [{"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True},
                {"Class": "rviz_default_plugins/TF", "Name": "TF", "Enabled": False}]
    for n in names:
        displays.append({
            "Class": "rviz_default_plugins/RobotModel", "Name": n, "Enabled": True,
            "Description Source": "Topic",
            "Description Topic": {"Value": f"/{n}/robot_description", "Depth": 5,
                                  "Durability Policy": "Transient Local",
                                  "History Policy": "Keep Last",
                                  "Reliability Policy": "Reliable"},
            "TF Prefix": n, "Visual Enabled": True, "Collision Enabled": False,
        })
    cfg = {"Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"}],
           "Visualization Manager": {
               "Class": "", "Displays": displays, "Enabled": True,
               "Global Options": {"Fixed Frame": "world", "Background Color": "48; 48; 48"},
               "Tools": [{"Class": "rviz_default_plugins/MoveCamera"}],
               "Views": {"Current": {"Class": "rviz_default_plugins/Orbit", "Distance": 6.0,
                                     "Focal Point": {"X": 0.0, "Y": 0.0, "Z": 0.3},
                                     "Pitch": 0.6, "Yaw": 0.8, "Name": "Current View"}}}}
    path = os.path.join(tempfile.gettempdir(), "cps_all_in_one.rviz")
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f)
    return path


def _setup(context):
    names = [n.strip() for n in context.launch_configurations["robots"].split(",") if n.strip()]
    spacing = float(context.launch_configurations["spacing"])
    nodes = []
    for i, n in enumerate(names):
        xml = u.read_urdf(u.urdf_path(n))
        y = (i - (len(names) - 1) / 2.0) * spacing
        nodes += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=n,
                 parameters=[{"robot_description": xml, "frame_prefix": f"{n}/"}]),
            Node(package="joint_state_publisher", executable="joint_state_publisher", namespace=n),
            Node(package="tf2_ros", executable="static_transform_publisher", name=f"world_to_{n}",
                 arguments=["--x", "0", "--y", str(y), "--z", "0",
                            "--frame-id", "world", "--child-frame-id", f"{n}/{u.root_link(xml)}"]),
        ]
    nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                      arguments=["-d", _rviz_config(names)]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value=DEFAULT),
        DeclareLaunchArgument("spacing", default_value="1.2"),
        OpaqueFunction(function=_setup),
    ])
