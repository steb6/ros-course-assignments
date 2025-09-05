# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Import ROS modules
import rclpy
from rclpy.node import Node

# Import ROS Messages and Services
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
import random

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        
        # Initialize the kill service client
        self.cli1 = self.create_client(Kill, 'kill')
        self.req1 = Kill.Request()
        
        # Initialize the spawn service client
        self.cli2 = self.create_client(Spawn, 'spawn')
        self.req2 = Spawn.Request()
        
        # Initialize publisher and subscriber
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle2/pose',
            self.listener_callback,
            10)
        
        # Wait for kill service to be available
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting again...')
        
        # Wait for spawn service to be available
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        
        
    def send_request(self):
        # Set request to kill turtle1
        self.req1.name = 'turtle1'
        self.future1 = self.cli1.call_async(self.req1)
        return self.future1
        
    def send_request2(self):
        # Set request to spawn a new turtle (turtle2)
        self.req2.x = random.uniform(1.0, 9.0)
        self.req2.y = random.uniform(1.0, 9.0) 
        self.req2.theta = random.uniform(0.0, 6.28)
        self.req2.name = 'turtle2'
        self.future2 = self.cli2.call_async(self.req2)
        return self.future2
        
    # Publish a velocity message when pose is received
    def listener_callback(self, msg):
        self.get_logger().info(f'Turtle2 position: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = 2.0  # Move forward
        twist.angular.z = 1.0  # Turn
        
        self.publisher.publish(twist)
        self.get_logger().info('Published velocity command to turtle2')


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    turtle_controller = TurtleController()
    
    # Call service 1 (kill turtle1)
    future1 = turtle_controller.send_request()
    rclpy.spin_until_future_complete(turtle_controller, future1)
    if future1.result() is not None:
        turtle_controller.get_logger().info('Successfully killed turtle1')
    else:
        turtle_controller.get_logger().error('Failed to kill turtle1')
    
    # Call service 2 (spawn turtle2)
    future2 = turtle_controller.send_request2()
    rclpy.spin_until_future_complete(turtle_controller, future2)
    if future2.result() is not None:
        turtle_controller.get_logger().info(f'Successfully spawned turtle2: {future2.result().name}')
    else:
        turtle_controller.get_logger().error('Failed to spawn turtle2')
    
    # Continue spinning to handle pose callbacks and publish velocities
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()