
import os
import json
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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


def generate_launch_description():
    
    share_dir = get_package_share_directory('ros2_webinterface')
    cofig_dir = os.path.join(share_dir, 'config')
    topic_list_yaml = os.path.join(cofig_dir, 'topic_list.yaml')
    topic_list_json = os.path.join(cofig_dir, 'topic_list.json')
    
    topic_list = create_yaml_from_json_file(topic_list_json, topic_list_yaml)
    

    webinterface = Node(
        package='ros2_webinterface',
        executable='webinterface',
        name='webinterface',
        output='screen',
        parameters=[
            topic_list_yaml
        ],
        arguments=['--ros-args', '--log-level', 'WARN'],
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