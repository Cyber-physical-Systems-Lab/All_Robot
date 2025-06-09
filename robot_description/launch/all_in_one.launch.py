from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Define robot types
    robot_models = [
        'mycomb',
        'nova5_robot_2f85',
        'mycomb1',
        'go2'
    ]
    
    urdf_path = get_package_share_path('robot_description')
    default_rviz_config_path = urdf_path / 'rviz/urdf.rviz'

    # Launch arguments
    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    # Create nodes for each robot
    robot_nodes = []
    model_args = []

    # Position offsets for each robot (x, y, z)
    positions = [
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (6.0, 0.0, 0.0)
    ]

    for i, (robot, position) in enumerate(zip(robot_models, positions)):
        # Define model path (without adding .urdf again)
        model_path = urdf_path / f'urdf/{robot}.urdf'
        print(f"Using URDF model for {robot} is: {model_path}")

        # Create unique model argument name for each robot
        model_arg_name = f'model_{i}'
        model_arg = DeclareLaunchArgument(
            name=model_arg_name, 
            default_value=str(model_path),
            description=f'Absolute path to {robot} urdf file'
        )
        model_args.append(model_arg)
        
        # Parse URDF with xacro
        robot_description = ParameterValue(
            Command(['xacro ', LaunchConfiguration(model_arg_name)]),
            value_type=str
        )
        
        # Robot state publisher with namespace
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=f'{robot}',
            name=f'robot_state_publisher_{robot}',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )

        # Joint state publisher with namespace
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=f'{robot}',
            name=f'joint_state_publisher_{robot}',
            condition=UnlessCondition(LaunchConfiguration('gui'))
        )

        # Static transform publisher to position each robot
        # static_transform_node = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name=f'static_transform_{robot}',
        #     arguments=[
        #         f'{position[0]}', f'{position[1]}', f'{position[2]}',  # x, y, z
        #         '0', '0', '0',  # roll, pitch, yaw
        #         'base_link',  # parent frame
        #         f'{robot}/base_link'
        #     ]
        # )
        
        # Add all nodes for this robot
        robot_nodes.extend([robot_state_publisher_node, joint_state_publisher_node]) #static_transform_node
    
    # Shared joint state publisher GUI (optional)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Return complete launch description with all components
    return LaunchDescription([
        gui_arg,
        rviz_arg
    ] + model_args + [
        joint_state_publisher_gui_node,
        rviz_node
    ] + robot_nodes)