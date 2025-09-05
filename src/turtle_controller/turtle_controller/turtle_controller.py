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

import random
import threading
import time

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_interfaces.msg import TurtleInfo


class TurtleController(Node):
    def __init__(self):
        self.__dict__['publishers'] = {}
        self.__dict__['turtle_poses'] = {}

        # self.publishers = {}
        super().__init__('turtle_controller')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        self.spawn_turtle("turtle2")

        # To publish velocity commands
        self.__dict__['publishers']['turtle1'] = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.__dict__['publishers']['turtle2'] = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        # self.publishers['turtle1'] = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # self.publishers['turtle2'] = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Subscribers for turtle poses
        self.create_subscription(Pose, '/turtle1/pose', lambda msg: self.pose_callback('turtle1', msg), 10)
        self.create_subscription(Pose, '/turtle2/pose', lambda msg: self.pose_callback('turtle2', msg), 10)

        # To publish TurtleInfo
        self.turtle_info_publisher = self.create_publisher(TurtleInfo, '/turtle_info', 10)

        threading.Thread(target=self.user_interface, daemon=True).start()

    def spawn_turtle(self, name):
        request = Spawn.Request()
        request.x = random.uniform(1.0, 9.0)
        request.y = random.uniform(1.0, 9.0)
        request.theta = random.uniform(0.0, 6.28)
        request.name = name
        self.get_logger().info(f'Spawning {name} at ({request.x:.2f}, {request.y:.2f}, {request.theta:.2f})')
        self.spawn_client.call_async(request)
        self.get_logger().info(f'{name} spawned.')

    def pose_callback(self, turtle_name, msg):
        self.__dict__['turtle_poses'][turtle_name] = msg

    def publish_turtle_info(self, turtle_name):
        """Publish TurtleInfo message with current turtle position"""
        turtle_poses = self.__dict__.get('turtle_poses', {})
        if turtle_name in turtle_poses:
            pose = turtle_poses[turtle_name]
            turtle_info = TurtleInfo()
            turtle_info.name = turtle_name
            turtle_info.x = pose.x
            turtle_info.y = pose.y
            turtle_info.theta = pose.theta
            self.turtle_info_publisher.publish(turtle_info)
            self.get_logger().info(f'Published TurtleInfo for {turtle_name}: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}')

    def user_interface(self):
        while rclpy.ok():
            # Access publishers through __dict__ to avoid generator issue
            publishers_dict = self.__dict__.get('publishers', {})
            print('\nAvailable turtles:', list(publishers_dict.keys()))
            turtle = input('Select turtle to control (turtle1/turtle2): ').strip()
            if turtle not in publishers_dict:
                print('Invalid turtle name.')
                continue
            try:
                lin = float(input('Enter linear velocity: '))
                ang = float(input('Enter angular velocity: '))
            except ValueError:
                print('Invalid input. Please enter numbers.')
                continue
            twist = Twist()
            twist.linear.x = lin
            twist.angular.z = ang
            publishers_dict[turtle].publish(twist)
            print(f'Sent command to {turtle}: linear={lin}, angular={ang} (for 1 second)')
            time.sleep(1)
            publishers_dict[turtle].publish(Twist())
            print(f'{turtle} stopped.')
            
            # Publish turtle info after movement
            self.publish_turtle_info(turtle)


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

