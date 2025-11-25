#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from lab4_interfaces.srv import Random
import random

import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3, UnitQuaternion
import numpy as np

class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')

        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)

        self.random_server = self.create_service(
            Random,
            "random_server",
            self.random_server_callback
        )

        self.r_max = 0.28 + 0.25
        self.r_min = 0.03
        self.l = 0.2
        
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0,    a=0.0,   d=0.2),
                rtb.RevoluteMDH(alpha=-pi/2,  a=0.0,   d=-0.12, offset=-pi/2),
                rtb.RevoluteMDH(alpha=0.0,    a=0.25,  d=0.1),
            ],
            
            tool=SE3.Tx(0.28) @ SE3.Rx(pi/2),
            name="3R_Robot",
        )

        self.get_logger().info("random_node has been started (Random only).")

    def random_server_callback(self, request, response):
        self.get_logger().info(f"Request mode = {request.mode}")

        if request.mode == "AUTO":
            max_attempts = 100
            for _ in range(max_attempts):
                x = random.uniform(-self.r_max, self.r_max)
                y = random.uniform(-self.r_max, self.r_max)
                z = random.uniform(-self.r_max, self.r_max)

                distance_squared = x**2 + y**2 + (z - self.l)**2
                if not (self.r_min**2 < distance_squared < self.r_max**2):
                    continue

                is_reachable, q_sol = self.check_reachability_and_get_q(x, y, z)
                
                if not is_reachable:
                    continue

                T_target = self.robot.fkine(q_sol)
                quat = UnitQuaternion(T_target.R)

                msg = PoseStamped()
                msg.header.frame_id = "link_0"
                msg.header.stamp = self.get_clock().now().to_msg()
                
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = z
                
                msg.pose.orientation.w = quat.s
                msg.pose.orientation.x = quat.v[0]
                msg.pose.orientation.y = quat.v[1]
                msg.pose.orientation.z = quat.v[2]

                self.target_pub.publish(msg)

                response.position.x = x
                response.position.y = y
                response.position.z = z
                response.inprogress = True
                return response
            
            self.get_logger().warn("Could not find valid random pose.")
            response.inprogress = False 

        return response

    def check_reachability_and_get_q(self, x, y, z):
        T_Position = SE3(x, y, z)

        ik = self.robot.ikine_LM(
            T_Position,
            mask=[1, 1, 1, 0, 0, 0],
            joint_limits=False,
            q0=[0, 0, 0]
        )
        return ik.success, ik.q

def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()