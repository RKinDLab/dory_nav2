import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
# from builtin_interfaces.msg import Time
# import time

class ClockPublisherNode(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        clock_msg = Clock()
        clock_msg.clock = current_time
        self.publisher_.publish(clock_msg)
        self.get_logger().info(f'Publishing: {current_time.sec}.{current_time.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
