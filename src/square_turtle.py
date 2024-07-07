import rclpy
from rclpy.node import Node
#import your message
from geometry_msgs.msg import Twist
import time
import math

class Square_Movement(Node):
	def __init__(self):
		super().__init__('square_turtle')
		self.me = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.move_turtle_square()
	
	def move_turtle_square(self):
		lin = Twist()
		ang = Twist()
		
		lin.linear.x = 1.0
		ang.angular.z = math.PI/2
		
		for i in range(4):
			self.me.publish(lin)
			time.sleep(1)
			
			self.me.publish(Twist())
			
			self.me.publsih(ang)
			time.sleep(1)
			
			self.me.publish(Twist())
		

def main(args=None):
	rclpy.init(args=args)
	node = Square_Movement()
	
	rclpy.spin(node)
	
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
