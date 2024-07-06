# ROS 2 Humble: Creating and Using Services

In this tutorial, we will create a simple ROS 2 service and client in Python using the ROS 2 Humble distribution. Services in ROS 2 allow for synchronous communication, where a node can request a task and wait for the result.

## Prerequisites

- ROS 2 Humble installed
- Basic understanding of ROS 2 concepts
- Python 3.8 or later

## Step 1: Create a New Package

First, create a new package for your service and client nodes.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_service_pkg
```

## Step 2: Define the Service

Create a new directory for your service definitions.

```bash
mkdir -p ~/ros2_ws/src/my_service_pkg/srv
```
Inside this directory, create a new file named AddTwoInts.srv with the following content:

```

int64 a
int64 b
---
int64 sum
```
This service takes two integers as input and returns their sum.

## Step 3: Update Package Files
`setup.py`
Update the setup.py file to include the service files.
```
from setuptools import setup

package_name = 'my_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/AddTwoInts.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karthik',
    maintainer_email='your_email@example.com',
    description='Examples of ROS2 services in Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = my_pub_sub.service:main',
            'client = my_pub_sub.client:main',
        ],
    },
)
```
### Now update the following file
`package.xml`
Update the package.xml to include the build dependencies.
```
<package format="3">
  <name>my_service_pkg</name>
  <version>0.0.0</version>
  <description>Examples of ROS2 services in Python</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <build_depend>ament_lint_auto</build_depend>
  <build_depend>ament_lint_common</build_depend>

  <build_depend>rclpy</build_depend>
  <build_depend>rosidl_default_runtime</build_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Step 4: Create the Service Node
Create a service.py file in the your_package directory.
```
import rclpy
from rclpy.node import Node
from my_service_pkg.srv import AddTwoInts

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}, sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 5: Create the Client Node
Create a client.py file in the your_package directory.
```
import sys
import rclpy
from rclpy.node import Node
from my_service_pkg.srv import AddTwoInts

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(f'Service call failed: {e}')
            else:
                node.get_logger().info(f'Result: {response.sum}')
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 6: Build the Package
Navigate to your workspace directory and build the package.
```
cd ~/ros2_ws
colcon build --packages-select your_package_name
```

Source the setup script to overlay your workspace on top of your environment.

```bash
source install/setup.bash
```

Step 7: Run the Service and Client
In one terminal, start the service node:
```
ros2 run your_package_name service
```

In another terminal, start the client node with two integer arguments:

```
ros2 run your_package_name client 3 5
```
You should see the client node send a request to the service node and receive the sum of the two integers.
