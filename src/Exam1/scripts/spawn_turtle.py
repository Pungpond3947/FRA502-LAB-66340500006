#!/usr/bin/python3

from Exam1.dummy_module import dummy_function, dummy_var
import rclpy
import sys
from rclpy.node import Node
from turtlesim.srv import Spawn
from std_msgs.msg import String
import numpy as np

class DummyNode(Node):
    def __init__(self):
        super().__init__('spawn_node')
        self.declare_parameter("name_teleop", "teleop")
        self.name_teleop = (
            self.get_parameter("name_teleop").get_parameter_value().string_value
        )
        self.declare_parameter("name_copy", "copy")
        self.name_copy = (
            self.get_parameter("name_copy").get_parameter_value().string_value
        )
        self.declare_parameter("turtle1", "Foxy")
        self.name1 = (
            self.get_parameter("turtle1").get_parameter_value().string_value
        )
        self.declare_parameter("turtle2", "Noetic")
        self.name2 = (
            self.get_parameter("turtle2").get_parameter_value().string_value
        )
        self.declare_parameter("turtle3", "Humble")
        self.name3 = (
            self.get_parameter("turtle3").get_parameter_value().string_value
        )
        self.declare_parameter("turtle4", "Iron")
        self.name4 = (
            self.get_parameter("turtle4").get_parameter_value().string_value
        )

        self.shutdown_sub = self.create_subscription(String, '/shutdown_signal', self.shutdown_callback, 10)

        self.spawn_turtle1_client = self.create_client(Spawn, self.name_teleop + "/spawn_turtle")
        self.spawn_turtle2_client = self.create_client(Spawn, self.name_copy + "/spawn_turtle")

        self.create_timer(1.0, self.spawn_turtle_once)

        self.spawn = False

    def shutdown_callback(self, msg):
        self.get_logger().info(f"{self.get_name()} ได้รับ shutdown signal")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def spawn_turtle(self, name, namespace):
        request = Spawn.Request()
        request.x = 0.1
        request.y = 0.1
        request.theta = 0.0
        request.name = name

        if namespace == self.name_teleop:
            if name == namespace:
                name = name[1:]
                request.name = name
            self.spawn_turtle1_client.call_async(request)
        elif namespace == self.name_copy:
            self.spawn_turtle2_client.call_async(request)

    def spawn_turtle_once(self):
        if self.spawn:
            return
        if not self.spawn_turtle1_client.service_is_ready() or not self.spawn_turtle2_client.service_is_ready():
            return
        self.spawn_turtle(self.name_teleop, self.name_teleop)
        self.spawn_turtle(self.name1, self.name_copy)
        self.spawn_turtle(self.name2, self.name_copy)
        self.spawn_turtle(self.name3, self.name_copy)
        self.spawn_turtle(self.name4, self.name_copy)
        self.spawn = True

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
