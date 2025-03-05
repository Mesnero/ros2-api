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
    
    config_file = LaunchConfiguration('config_file')
    
    node_server = Node(
        package='core',
        executable='ros2_api_node',
        output='screen',
        arguments=['--config_file', config_file]
    )

    return LaunchDescription([config_file_arg, node_server])