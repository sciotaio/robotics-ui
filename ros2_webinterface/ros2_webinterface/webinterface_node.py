import rclpy
from rclpy.node import Node
import std_msgs.msg
import sensor_msgs.msg

from threading import Lock, Thread
from rclpy.executors import MultiThreadedExecutor

from fastapi import FastAPI
from fastapi.responses import JSONResponse
import uvicorn
from fastapi import Depends

import json
from rosidl_runtime_py import message_to_ordereddict, convert

from enum import Enum
from .util.message_type_enum import MessageType, get_class_from_enum_value

import os
import yaml
from ament_index_python.packages import get_package_share_directory


app = FastAPI()

# Queue and lock for sharing data between ROS node and FastAPI application
String_rout_queue = {}
String_rout_lock = Lock()
rout_queue = {}
rout_lock = Lock()


class SubscriberList:
    def __init__(self, name, topic, type, subscriber):
        self.name = name
        self.topic = topic
        self.type = type
        self.subscriber = subscriber

class SubscriberWebinterface(Node):
    def __init__(self):
        super().__init__('subscriber_webinterface')

        self.declare_parameter('subscriber_list_name')
        self.declare_parameter('subscriber_list_topic')
        self.declare_parameter('subscriber_list_type')
        
        self.topic_list_name = self.get_parameter('subscriber_list_name').get_parameter_value().string_array_value
        self.topic_list_topic = self.get_parameter('subscriber_list_topic').get_parameter_value().string_array_value
        self.topic_list_type = self.get_parameter('subscriber_list_type').get_parameter_value().string_array_value     

        self.subscriber_list = []
        self.subscriber_list_index = 0
        
        self.startup_subscriber()   
    
    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
            
    def startup_subscriber(self):
        
        self.get_logger().info("Starting subscriber")
        for i in range(len(self.topic_list_name)):
            self.start_subscriber(self.topic_list_name[i], self.topic_list_topic[i], self.topic_list_type[i], i)
    
    
    def start_subscriber(self, topic_name, topic_topic, topic_type, index):        
        
        self.get_logger().info(f"Starting subscriber {topic_name} with topic {topic_topic} and type {topic_type}")
        
        self.subscriber_list_index = index        
        message_type = get_class_from_enum_value(topic_type)
        
        if message_type is not None:        
            subscriber = self.create_subscription(
                    message_type,
                    topic_topic,  
                    lambda msg, index=self.subscriber_list_index:self.subscriber_callback(msg, index),
                    1
                )
            self.subscriber_list.append(SubscriberList(topic_name, topic_topic, topic_type, subscriber))
            
        else:
            self.get_logger().error(f"Message type {topic_type} not found")      
            
    
    def stop_subscriber(self):
        
        self.get_logger().info("Stopping subscriber")
        for i in range(len(self.subscriber_list)):
            self.destroy_subscription(self.subscriber_list[i].subscriber)
            
    
    def update_subscriber_parameters(self, topic_list_name, topic_list_topic, topic_list_type):
        
        self.get_logger().info(f"Updating subscriber parameters: name={topic_list_name}, topic={topic_list_topic}, type={topic_list_type}")
        self.topic_list_name = topic_list_name
        self.topic_list_topic = topic_list_topic
        self.topic_list_type = topic_list_type
    

    def subscriber_callback(self, msg, index):

        message_dict = self.ros2_message_to_json(msg)        
        with rout_lock:
            rout_queue[self.subscriber_list[index].name] = message_dict

                
    def ros2_message_to_json(self, message):
        message_dict = message_to_ordereddict(message)
        # self.get_logger().info(f"message_dict: {message_dict}")
        # message_json = json.dumps(message_dict)
        return message_dict
    

class PublisherList:
    def __init__(self, name, topic, type, publisher):
        self.name = name
        self.topic = topic
        self.type = type
        self.publisher = publisher

class PublisherWebinterface(Node):
    def __init__(self):
        super().__init__('publisher_webinterface')
        
        self.declare_parameter('publisher_list_name')
        self.declare_parameter('publisher_list_topic')
        self.declare_parameter('publisher_list_type')
        
        self.topic_list_name = self.get_parameter('publisher_list_name').get_parameter_value().string_array_value
        self.topic_list_topic = self.get_parameter('publisher_list_topic').get_parameter_value().string_array_value
        self.topic_list_type = self.get_parameter('publisher_list_type').get_parameter_value().string_array_value

        self.publisher_list = []
        self.publisher_list_index = 0
        
        self.startup_publisher()
        

    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    
    def startup_publisher(self):
        
        self.get_logger().info("Starting publisher")
        for i in range(len(self.topic_list_name)):
            self.start_publisher(self.topic_list_name[i], self.topic_list_topic[i], self.topic_list_type[i], i)
    
    
    def start_publisher(self, topic_name, topic_topic, topic_type, index):        
        
        self.get_logger().info(f"Starting publisher {topic_name} with topic {topic_topic} and type {topic_type}")
        self.publisher_list_index = index        
        message_type = get_class_from_enum_value(topic_type)
        
        if message_type is not None:        
            publisher = self.create_publisher(
                message_type,
                topic_topic,  
                1
            )
            self.publisher_list.append(PublisherList(topic_name, topic_topic, topic_type, publisher))
            
        else:
            self.get_logger().error(f"Message type {topic_type} not found")

    
    def stop_publisher(self):
        
        self.get_logger().info("Stopping publisher")
        for i in range(len(self.publisher_list)):
            self.destroy_publisher(self.publisher_list[i].publisher)
    
    def update_publisher_parameters(self, topic_list_name, topic_list_topic, topic_list_type):
        
        self.get_logger().info(f"Updating publisher parameters: name={topic_list_name}, topic={topic_list_topic}, type={topic_list_type}")
        self.topic_list_name = topic_list_name
        self.topic_list_topic = topic_list_topic
        self.topic_list_type = topic_list_type
            
    
    def trigger(self, data, publisher_name):
        for publisher in self.publisher_list:
            if publisher.name == publisher_name:
                                
                if publisher.type == 'String':                    
                    self.get_logger().info(f"Publisher {publisher_name} received: {data}")
                    
                    msg = std_msgs.msg.String()
                    msg.data = json.dumps(data)
                    publisher.publisher.publish(msg)
                return
        
def get_publisher_webinterface():
    return PublisherWebinterface.get_instance()

def get_subscriber_webinterface():
    return SubscriberWebinterface.get_instance()


def update_webinterface_config(data, subscriber_webinterface, publisher_webinterface):
    
    ui_test_share = get_package_share_directory('ui_test')
    cofig_dir = os.path.join(ui_test_share, 'config')
    topic_list_json = os.path.join(cofig_dir, 'topic_list.json')
    
    json_data = json.dumps(data)    
    with open(topic_list_json, 'w') as json_file:
        json_data
    
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
    
    subscriber_webinterface.update_subscriber_parameters(subscriber_list_name, subscriber_list_topic, subscriber_list_type)
    publisher_webinterface.update_publisher_parameters(publisher_list_name, publisher_list_topic, publisher_list_type)


@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/subscriber/{subscriber_name}")
def read(subscriber_name):
    with rout_lock:
        # Fetch and return messages from the queue
        message = rout_queue.get(subscriber_name)
    return JSONResponse(content=message)

@app.post("/publisher/{publisher_name}")
def write(publisher_name: str, message: dict, publisher_webinterface: PublisherWebinterface = Depends(get_publisher_webinterface)):
    publisher_webinterface.trigger(message, publisher_name)
    return 'Success', 200

@app.post("/settings/update_json_config")
def update_json_config(data: dict, publisher_webinterface: PublisherWebinterface = Depends(get_publisher_webinterface), subscriber_webinterface: SubscriberWebinterface = Depends(get_subscriber_webinterface)):
    update_webinterface_config(data, subscriber_webinterface, publisher_webinterface)    
    return 'Success', 200


@app.post("/settings/manage_webinterface")
def update_json_config(comand: dict, publisher_webinterface: PublisherWebinterface = Depends(get_publisher_webinterface), subscriber_webinterface: SubscriberWebinterface = Depends(get_subscriber_webinterface)):
    if comand['action'] == 'restart':
        subscriber_webinterface.stop_subscriber()
        publisher_webinterface.stop_publisher()    
        subscriber_webinterface.startup_subscriber()
        publisher_webinterface.startup_publisher()   
        return 'Success', 200
    elif comand['action'] == 'stop':
        subscriber_webinterface.stop_subscriber()
        publisher_webinterface.stop_publisher()
        return 'Success', 200

def start_fastapi():
    uvicorn.run(app, host="0.0.0.0", port=8000)


def main():
    
    rclpy.init()

    subscriber_webinterface = SubscriberWebinterface()
    publisher_webinterface = PublisherWebinterface()    
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(subscriber_webinterface)
    executor.add_node(publisher_webinterface)

    # Start the FastAPI server in a separate thread
    fastapi_thread = Thread(target=start_fastapi)
    fastapi_thread.start()    

    try:        
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    subscriber_webinterface.destroy_node()
    publisher_webinterface.destroy_node()
    
    rclpy.shutdown()
    fastapi_thread.join()

if __name__ == '__main__':
    main()