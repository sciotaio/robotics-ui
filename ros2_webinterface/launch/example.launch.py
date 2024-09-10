import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    share_dir = get_package_share_directory('ros2_webinterface')    
    launch_dir = os.path.join(share_dir, 'launch')
    
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
    ld.add_action(webinterface)
    return ld