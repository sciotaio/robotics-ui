import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from woodbot_pkg.utils_gps import latLonYaw2Geopose
from std_msgs.msg import String
import json

import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion
import threading
from rclpy.executors import MultiThreadedExecutor


class Waypoint:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
    
    def return_geopoint(self):
        return latLonYaw2Geopose(latitude=self.latitude, longitude=self.longitude, yaw=0.0)
    
    def return_json(self):
        return json.dumps({'marker': {'lat': self.latitude, 'lng': self.longitude}})

class WaypointManager(Node):
    """
    ROS2 node to manage and send gps waypoints to nav2 received from robotics-ui
    """

    def __init__(self, navigator):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = navigator
                
        self.waypoint_list = []
        self.lock = threading.Lock()
        self.update_event = threading.Event()
        self.delete_event = threading.Event()
        self.runing_task = threading.Event()
        self.thread = threading.Thread(target=self.send_waypoints)
        self.thread.start()
        
        self.navigator.waitUntilNav2Active(localizer='robot_localization')


        self.subscriber = self.create_subscription(
            String,
            "/waypoint_add",
            self.waypoint_add_callback,
            1
        )
        
        self.subscriber = self.create_subscription(
            String,
            "/waypoint_delete",
            self.waypoint_delete_callback,
            1
        )

        self.waypoint_reached_pub = self.create_publisher(String, "/waypoint_reached", 1)

    def waypoint_add_callback(self, msg):
        markers_res = json.loads(msg.data)

        latitude = markers_res['marker']['lat']
        longitude = markers_res['marker']['lng']
        
        with self.lock:
            self.waypoint_list.append(Waypoint(latitude, longitude))
        
        self.get_logger().info(f"Waypoint list: {self.waypoint_list}")
        
        if self.update_event.is_set() == False:
            self.update_event.set()
        

    def waypoint_delete_callback(self, msg):
        markers_res = json.loads(msg.data)
        
        latitude = markers_res['marker']['lat']
        longitude = markers_res['marker']['lng']
        
        for i in range(len(self.waypoint_list)):
            
            if self.waypoint_list[i].latitude == latitude and self.waypoint_list[i].longitude == longitude:
                if i == 0:
                    self.delete_event.set()
                else:
                    with self.lock:
                        self.waypoint_list.pop(i)
                        self.get_logger().info(f"Waypoint deleted")    
        
    
    def send_waypoints(self):
         
        self.current_waypoint = 0
         
        while True:
            
            if self.runing_task.is_set() == False:
                    self.update_event.wait()                    
                    if len(self.waypoint_list) > 0:
                        with self.lock:
                            self.current_waypoint = self.waypoint_list[0]
                        self.runing_task.set()
                        self.get_logger().info(f"Waypoint sent to nav2")
                        waypoint = [self.current_waypoint.return_geopoint()]
                        self.get_logger().info(f"Waypoint: {waypoint}")
                        self.navigator.followGpsWaypoints(waypoint)
                    else:
                        self.update_event.clear()
                        continue
            
            if self.runing_task.is_set():
                if self.navigator.isTaskComplete():
                    self.get_logger().info(f"Waypoint reached")    
                    self.runing_task.clear()
                    with self.lock:
                        # Remove the old waypoint
                        self.waypoint_list.pop(0)
                        
                        # send msg to frontend to remove the old waypoint                        
                        msg = String()
                        msg.data = self.current_waypoint.return_json()
                        self.waypoint_reached_pub.publish(msg)
                    


def main():

    rclpy.init()
    navigator = BasicNavigator("basic_navigator")
    waypoint_manager = WaypointManager(navigator)
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(navigator)
    executor.add_node(waypoint_manager)

    try:        
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
