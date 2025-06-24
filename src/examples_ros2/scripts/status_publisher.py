import rclpy
from rclpy.node import Node
from examples_ros2.msg import Status
import argparse
import time

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(Status, 'status', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.id = 0
        
    def timer_callback(self):
        msg = Status()
        msg.status = f'completed_{self.id}'
        msg.id = self.id
        msg.timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending status: {msg.status} with id: {msg.id}')
        self.id += 1

def main():
    rclpy.init()
    node = StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
