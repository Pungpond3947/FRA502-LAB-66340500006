#!/usr/bin/python3

from lab3.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class DummyNode(Node):
    def __init__(self):
        super().__init__('SetParam')
        self.kp_linear_pub = self.create_publisher(Int64, "/kp_linear", 10)
        self.kp_angular_pub = self.create_publisher(Int64, "/kp_angular", 10)
        self.MaxPizza_pub = self.create_guard_condition(Int64, "/max_pizza", 10)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
