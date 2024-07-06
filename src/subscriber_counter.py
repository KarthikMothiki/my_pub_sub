# my_pub_sub/src/subscriber_counter.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterSubscriber(Node):

    def __init__(self):
        super().__init__('counter_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'counter_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    counter_subscriber = CounterSubscriber()
    rclpy.spin(counter_subscriber)

    # Destroy the node explicitly
    counter_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
