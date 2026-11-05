"""RViz2 viewer for one robot.

  ros2 launch cps_robot_description display.launch.py robot:=nova5_2f85
  ros2 launch cps_robot_description display.launch.py robot:=mycobot_280_pi variant:=with_pump gui:=true
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cps_launch_utils as u  # noqa: E402


def _setup(context):
    robot = context.launch_configurations["robot"]
    variant = context.launch_configurations["variant"]
    gui = context.launch_configurations["gui"] == "true"
    rviz_cfg = context.launch_configurations["rviz_config"] or os.path.join(u.share(), "rviz", "urdf.rviz")
    xml = u.read_urdf(u.urdf_path(robot, variant))
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": xml}]),
        Node(package="joint_state_publisher_gui" if gui else "joint_state_publisher",
             executable="joint_state_publisher_gui" if gui else "joint_state_publisher"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", rviz_cfg, "-f", u.root_link(xml)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="nova5_2f85",
                              description="robot name from robots.yaml"),
        DeclareLaunchArgument("variant", default_value="",
                              description="substring of a variant URDF file name"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="custom .rviz file (default: rviz/urdf.rviz)"),
        OpaqueFunction(function=_setup),
    ])
