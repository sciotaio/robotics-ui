
import os
import json
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from ament_index_python.packages import get_package_share_directory

#=====================================================
# This launch file is used to test the webinterface 
#-----------------------------------------------------
# includes test publisher and subscriber to generate test data
#=====================================================

def generate_launch_description():
    
    share_dir = get_package_share_directory('ros2_webinterface')    
    launch_dir = os.path.join(share_dir, 'launch')    
    
    webinterface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'webinterface.launch.py')
        ),
        launch_arguments={
            "yaml_config": 'topic_list.yaml',
            "json_config": 'topic_list.json',
            "config_folder": 'default',         # default for the standart file path, althernatively fixed file path
        }.items(),
    )
    
    test_publischer = Node(
        package='ros2_webinterface',
        executable='test_publischer',
        name='test_publischer',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
    )
    
    test_subscriber = Node(
        package='ros2_webinterface',
        executable='test_subscriber',
        name='test_subscriber',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
    )
    
    waypoint_manager = Node(
        package='ros2_webinterface',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
    )
    
    ld = LaunchDescription()
    
    ld.add_action(webinterface)
    ld.add_action(test_publischer)
    ld.add_action(test_subscriber)
    ld.add_action(waypoint_manager)

    return ld