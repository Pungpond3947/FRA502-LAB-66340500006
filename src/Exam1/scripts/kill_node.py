#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ManagerNode(Node):
    def __init__(self, worker_names):
        super().__init__('killnode_node')
        self.worker_names = set(worker_names)  # list ของ node ที่ต้องรอ
        self.done_nodes = set()
        self.status_sub = self.create_subscription(String, '/status', self.status_callback, 10)
        self.shutdown_pub = self.create_publisher(String, '/shutdown_signal', 10)

    def status_callback(self, msg):
        self.get_logger().info(f"Manager ได้รับ status: {msg.data}")
        self.done_nodes.add(msg.data)
        if self.done_nodes == self.worker_names:
            self.get_logger().info("ทุก node ทำงานเสร็จ → ส่ง shutdown signal")
            shutdown_msg = String()
            shutdown_msg.data = "shutdown_all"
            self.shutdown_pub.publish(shutdown_msg)
            # ปิดตัวเองด้วย
            self.get_logger().info("Manager node กำลังปิดตัวเอง")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    manager = ManagerNode(['teleop_node', 'kill_node', 'spawn_node', 'eraser_node', 'copy_turtle_node']) 
    rclpy.spin(manager)
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
