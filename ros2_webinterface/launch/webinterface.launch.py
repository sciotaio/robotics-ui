
import os
import json
import yaml

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution



from ament_index_python.packages import get_package_share_directory
# from .parameter_converter import create_yaml_from_json_file


def create_yaml_from_json_file(json_file_path, yaml_file_path):
    # Read JSON file
    with open(json_file_path, 'r') as json_file:
        json_data = json.load(json_file)

    # Extract subscriber and publisher information
    subscriber_list_name = []
    subscriber_list_topic = []
    subscriber_list_type = []
    publisher_list_name = []
    publisher_list_topic = []
    publisher_list_type = []

    # Initialize sets to keep track of unique entries
    unique_subscribers = set()
    unique_publishers = set()

    for component in json_data['component']:
        for topic in component['topic']:
            if topic['route'].startswith('/subscriber'):
                subscriber_key = (topic['name'], topic['topic'], topic['type'])
                if subscriber_key not in unique_subscribers:
                    unique_subscribers.add(subscriber_key)
                    subscriber_list_name.append(topic['name'])
                    subscriber_list_topic.append(topic['topic'])
                    subscriber_list_type.append(topic['type'])
            elif topic['route'].startswith('/publisher'):
                publisher_key = (topic['name'], topic['topic'], topic['type'])
                if publisher_key not in unique_publishers:
                    unique_publishers.add(publisher_key)
                    publisher_list_name.append(topic['name'])
                    publisher_list_topic.append(topic['topic'])
                    publisher_list_type.append(topic['type'])

    # Create the data structure
    data = {
        'webinterface': {
            'ros__parameters': {
                'subscriber_list_name': subscriber_list_name,
                'subscriber_list_topic': subscriber_list_topic,
                'subscriber_list_type': subscriber_list_type,
                'publisher_list_name': publisher_list_name,
                'publisher_list_topic': publisher_list_topic,
                'publisher_list_type': publisher_list_type
            }
        }
    }

    # Write to YAML file
    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)
    
    # Convert the data structure to a YAML-formatted string
    yaml_data = yaml.dump(data, default_flow_style=False)

    return yaml_data

def launch_setup(context: LaunchContext, *args):
    yaml_config, json_config, config_folder = args
    # Convert LaunchConfigurations to strings
    yaml_config_str = context.perform_substitution(yaml_config)
    json_config_str = context.perform_substitution(json_config)
    config_folder_str = context.perform_substitution(config_folder)
    
    if (config_folder_str == 'default'):
        share_dir = get_package_share_directory('ros2_webinterface')
        config_dir = os.path.join(share_dir, 'config')
    else:
        config_dir = config_folder_str
    
    print(type(yaml_config_str))    
    
    topic_list_yaml = os.path.join(config_dir, yaml_config_str)
    topic_list_json = os.path.join(config_dir, json_config_str)    
    
    print(topic_list_yaml)
    
    topic_list = create_yaml_from_json_file(topic_list_json, topic_list_yaml) 
    
    webinterface = Node(
        package='ros2_webinterface',
        executable='webinterface',
        name='webinterface',
        output='screen',
        parameters=[topic_list_yaml]
    )
    
    return [webinterface]


def generate_launch_description():
    # Declare launch arguments
    yaml_config_launch_arg = DeclareLaunchArgument(
        'yaml_config',
        default_value='topic_list.yaml',
        description='YAML config file name'
    )

    json_config_launch_arg = DeclareLaunchArgument(
        'json_config',
        default_value='topic_list.json',
        description='JSON config file name'
    )

    # Use LaunchConfiguration to get the argument values
    yaml_config = LaunchConfiguration('yaml_config')
    json_config = LaunchConfiguration('json_config')
    config_folder = LaunchConfiguration('config_folder') 
    
    
    launch_setup_action = OpaqueFunction(function=launch_setup, args=[yaml_config, json_config, config_folder])

    ld = LaunchDescription()
    
    ld.add_action(yaml_config_launch_arg)
    ld.add_action(json_config_launch_arg)
    ld.add_action(launch_setup_action)
    
    return ld