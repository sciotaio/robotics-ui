import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Publischer(Node):
    def __init__(self):
        super().__init__('image_streamer')

        self.publisher_1 = self.create_publisher(String, '/topic1', 1)
        self.publisher_2 = self.create_publisher(String, '/topic2', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_1.publish(msg)
        self.publisher_2.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    
        


def main():
    rclpy.init()
    publischer = Publischer()
    

    try:
        rclpy.spin(publischer)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()