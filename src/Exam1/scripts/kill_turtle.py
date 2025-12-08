#!/usr/bin/python3

from Exam1.dummy_module import dummy_function, dummy_var
import rclpy
import sys
from rclpy.node import Node
from turtlesim.srv import Kill
from std_msgs.msg import String
import numpy as np

class DummyNode(Node):
    def __init__(self):
        super().__init__('killturtle_node')
        self.declare_parameter("name_teleop", "teleop")
        self.name_teleop = (
            self.get_parameter("name_teleop").get_parameter_value().string_value
        )
        self.declare_parameter("name_copy", "copy")
        self.name_copy = (
            self.get_parameter("name_copy").get_parameter_value().string_value
        )

        self.shutdown_sub = self.create_subscription(String, '/shutdown_signal', self.shutdown_callback, 10)

        self.kill_turtle1_client = self.create_client(Kill, self.name_teleop + "/remove_turtle")
        self.kill_turtle2_client = self.create_client(Kill, self.name_copy + "/remove_turtle")

        self.create_timer(1.0, self.kill_turtle_once)

        self.kill = False

    def shutdown_callback(self, msg):
        self.get_logger().info(f"{self.get_name()} ได้รับ shutdown signal")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def kill_turtle(self, name):
        kill = Kill.Request()
        kill.name = "turtle1"
        if name == self.name_teleop:
            self.kill_turtle1_client.call_async(kill)
        elif name == self.name_copy:
            self.kill_turtle2_client.call_async(kill)

    def kill_turtle_once(self):
        if self.kill:
            return
        if not self.kill_turtle1_client.service_is_ready() or not self.kill_turtle2_client.service_is_ready():
            return
        self.kill_turtle(self.name_teleop)
        self.kill_turtle(self.name_copy)
        self.kill = True

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
