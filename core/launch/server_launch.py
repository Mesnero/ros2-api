from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([FindPackageShare('robco_bringup'), 'config', 'api_config.yaml']),
        description='Path to the configuration file'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    node_server = Node(
        package='core',
        executable='ros2_api_node',
        output='screen',
        arguments=['--config_file', config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([config_file_arg, use_sim_arg, node_server])