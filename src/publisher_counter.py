# my_pub_sub/src/publisher_counter.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterPublisher(Node):

    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher_ = self.create_publisher(Int32, 'counter_topic', 10)
        self.counter = 0
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    counter_publisher = CounterPublisher()
    rclpy.spin(counter_publisher)

    # Destroy the node explicitly
    counter_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
