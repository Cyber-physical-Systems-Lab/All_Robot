import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    urdf_package = LaunchConfiguration('urdf_package')
    urdf_package_arg = DeclareLaunchArgument(
        'urdf_package', 
        default_value='robot_description',
        description='Package containing the URDF'
    )
    
    urdf_file = LaunchConfiguration('urdf_file')
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='urdf/your_robot.urdf',  # Update with your actual URDF file path
        description='Path to URDF file, relative to package'
    )
    
    # Set the spacing between robots (in meters)
    robot_spacing = 2.0  # Increase this value to spread robots further apart
    
    # Nodes to launch
    nodes = []
    
    # For each robot, create necessary nodes
    for i in range(6):
        # Create a unique namespace for each robot
        namespace = f'robot_{i+1}'
        
        # Calculate position - position robots in a line with proper spacing
        x_pos = i * robot_spacing
        
        # Static transform from world to robot base
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{i}',
                arguments=[str(x_pos), '0', '0', '0', '0', '0', 'world', f'{namespace}/base_link']
            )
        )
        
        # Load the robot description for this robot instance
        urdf_path = PathJoinSubstitution([FindPackageShare(urdf_package), urdf_file])
        robot_description = Command(['cat ', urdf_path])
        
        # Robot state publisher for this robot instance
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                parameters=[{'robot_description': robot_description,
                             'frame_prefix': f'{namespace}/'  # Important! Prefix all frames
                           }],
                output='screen'
            )
        )
        
        # Joint state publisher for this robot (if needed)
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=namespace
            )
        )
    
    # Add RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(urdf_package),
        'rviz/multi_robot.rviz'  # Create this configuration file
    ])
    
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    )
    
    # Create launch description with all nodes
    return LaunchDescription([
        urdf_package_arg,
        urdf_file_arg,
        *nodes
    ])