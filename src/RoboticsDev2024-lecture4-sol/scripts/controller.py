#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from lab4_interfaces.srv import SetMode

import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from math import pi

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0,    a=0.0,   d=0.2),
                rtb.RevoluteMDH(alpha=-pi/2,  a=0.0,   d=-0.12, offset=-pi/2),
                rtb.RevoluteMDH(alpha=0.0,    a=0.25,  d=0.1),
            ],
            tool=SE3.Tx(0.28),
            name="3R_Robot",
        )
        
        self.mode = "IDLE" # IPK, TELEOP, Auto
        self.q = np.array([0.0, -0.5, 1.0])
        self.dt = 0.02

        self.target_q = None
        self.start_q = None
        self.move_timer = 0.0
        self.move_duration = 10.0

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.ee_pub = self.create_publisher(PoseStamped, '/end_effector', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.srv = self.create_service(SetMode, 'robot_command', self.handle_command)

        self.teleop_vel = np.zeros(6)
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Controller: Waiting for commands...")

    def handle_command(self, request, response):
        self.get_logger().info(f"Controller: Received command {request.mode}")

        if request.mode == "IPK":
            x, y, z = request.target_pose.position.x, request.target_pose.position.y, request.target_pose.position.z
            
            q_sol = self.solve_ik(x, y, z)

            if q_sol is not None:
                self.start_q = np.copy(self.q)
                self.target_q = q_sol
                self.move_timer = 0.0
                self.mode = "MOVING"
                
                response.success = True
                response.configuration = q_sol.tolist()
            else:
                response.success = False
        
        elif request.mode == "TELEOP":
            self.mode = "TELEOP"
            response.success = True

        return response

    def control_loop(self):
        if self.mode == "MOVING":
            self.process_trajectory()
        elif self.mode == "TELEOP":
            self.process_teleop()

        self.publish_state()

    def process_trajectory(self):
        if self.target_q is None: return
        
        if self.move_timer < self.move_duration:
            progress = self.move_timer / self.move_duration
            self.q = self.start_q + (self.target_q - self.start_q) * progress
            self.move_timer += self.dt
        else:
            self.q = self.target_q
            self.mode = "IDLE"

    def process_teleop(self):
        if hasattr(self, 'use_ee_frame') and self.use_ee_frame:
             J = self.robot.jacobe(self.q) 
        else:
             J = self.robot.jacob0(self.q)
        
        J_pos = J[0:3, :] 

        try:
            det_J = np.linalg.det(J_pos)
        except np.linalg.LinAlgError:
            det_J = 0.0

        if abs(det_J) < 0.0005: 
            self.get_logger().warn(f"Singularity Detected! (Det: {det_J:.5f}) -> Bouncing to Safe Pose.")
            
            self.q = np.array([0.0, -0.5, 1.0]) 
            
            # หยุดความเร็วเพื่อตั้งหลักใหม่
            self.teleop_vel = np.zeros(6)
            return

        v_pos = self.teleop_vel[0:3]
        try:
            q_dot = np.linalg.pinv(J_pos) @ v_pos
            self.q = self.q + (q_dot * self.dt)
        except:
            pass

    def cmd_vel_callback(self, msg):
        self.teleop_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z, 0, 0, 0])

        # 0.0 = World, 1.0 = End-Effector
        self.use_ee_frame = (msg.angular.z > 0.5)

    def solve_ik(self, x, y, z):
        T = SE3(x, y, z)
        ik = self.robot.ikine_LM(T, mask=[1,1,1,0,0,0], q0=self.q)
        return ik.q if ik.success else None

    def publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["joint_1", "joint_2", "joint_3"]
        js.position = self.q.tolist()
        self.joint_pub.publish(js)

        T_ee = self.robot.fkine(self.q)
        ee = PoseStamped()
        ee.header.frame_id = "link_0"
        ee.pose.position.x = T_ee.t[0]
        ee.pose.position.y = T_ee.t[1]
        ee.pose.position.z = T_ee.t[2]
        self.ee_pub.publish(ee)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()