"""Spawn one robot in Gazebo Fortress (ROS 2 Humble + ros_gz).

  sudo apt install ros-humble-ros-gz
  ros2 launch cps_robot_description gazebo.launch.py robot:=nova5_2f85
  ros2 launch cps_robot_description gazebo.launch.py robot:=go2 world:=empty.sdf

Arms (fixed_base: true in robots.yaml) are welded to the world, mobile robots are free.
Joint control is not included: add gz_ros2_control tags to drive joints.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cps_launch_utils as u  # noqa: E402


def _setup(context):
    robot = context.launch_configurations["robot"]
    variant = context.launch_configurations["variant"]
    world = context.launch_configurations["world"]
    cfg = u.registry()[robot]
    xml = u.read_urdf(u.urdf_path(robot, variant))
    z = float(cfg.get("spawn_z", 0.0))
    if cfg.get("fixed_base", True):
        xml = u.weld_to_world(xml)
        z = 0.0

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world}"}.items())
    return [
        gz,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": xml, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", robot, "-z", str(z)]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]),
    ]


def generate_launch_description():
    # Gazebo resolves package://cps_robot_description/... by searching <install>/share
    share_parent = os.path.dirname(get_package_share_directory("cps_robot_description"))
    res = share_parent + os.pathsep + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    return LaunchDescription([
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", res),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res),
        DeclareLaunchArgument("robot", default_value="nova5_2f85"),
        DeclareLaunchArgument("variant", default_value=""),
        DeclareLaunchArgument("world", default_value="empty.sdf"),
        OpaqueFunction(function=_setup),
    ])
