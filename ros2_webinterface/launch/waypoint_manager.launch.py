
import os
import json
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
# from .parameter_converter import create_yaml_from_json_file


def generate_launch_description():
    
    share_dir = get_package_share_directory('ros2_webinterface')
    
    launch_dir = os.path.join(share_dir, 'launch')
        
    waypoint_manager = Node(
        package='ros2_webinterface',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
    )
    
    webinterface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'webinterface.launch.py')
        ),
        launch_arguments={
            "yaml_config": "topic_list.yaml",
            "json_config": "topic_list.json",
            "config_folder": 'default',         # default for the standart file path, althernatively fixed file path
        }.items(),
    )
    
    ld = LaunchDescription()

    ld.add_action(waypoint_manager)
    ld.add_action(webinterface)

    return ld