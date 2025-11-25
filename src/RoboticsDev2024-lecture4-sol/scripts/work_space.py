#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2 # ต้องใช้ module นี้ช่วยสร้าง point cloud

import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from math import pi
import random

class WorkspaceVisualizer(Node):
    def __init__(self):
        super().__init__('workspace_visualizer')
        
        self.pcd_pub = self.create_publisher(PointCloud2, '/workspace_points', 10)
        
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0,    a=0.0,   d=0.2),
                rtb.RevoluteMDH(alpha=-pi/2,  a=0.0,   d=-0.12, offset=-pi/2),
                rtb.RevoluteMDH(alpha=0.0,    a=0.25,  d=0.1),
            ],
            tool=SE3.Tx(0.28),
            name="3R_Robot",
        )

        self.q_min = -3.1459
        self.q_max = 3.1459

        self.create_timer(1.0, self.publish_workspace)
        self.get_logger().info("Workspace Node Started. Preparing to generate points...")

    def publish_workspace(self):
        points = []
        num_points = 3000

        for i in range(num_points):
            # 1. สุ่ม Joint Configuration ที่เป็นไปได้ (FK ง่ายกว่าและชัวร์กว่า IK)
            q1 = random.uniform(self.q_min, self.q_max)
            q2 = random.uniform(self.q_min, self.q_max)
            q3 = random.uniform(self.q_min, self.q_max)
            
            # 2. คำนวณ Forward Kinematics
            T = self.robot.fkine([q1, q2, q3])
            
            # 3. เก็บพิกัด (x, y, z)
            points.append([T.t[0], T.t[1], T.t[2]])

        # 4. สร้าง Message PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "link_0"  # อ้างอิงจากฐานหุ่นยนต์

        # แปลง list เป็น PointCloud2 msg
        pc2_msg = pc2.create_cloud_xyz32(header, points)
        
        # 5. Publish
        self.pcd_pub.publish(pc2_msg)
        # self.get_logger().info(f"Published workspace with {num_points} points.")
        
        # หยุด Timer เพราะเราโชว์แค่ครั้งเดียวก็พอ (หรือจะปล่อยให้ loop ก็ได้)
        # self.destroy_timer(self.timer) # ถ้าอยากให้รันครั้งเดียว

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()