#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point, TransformStamped
from std_msgs.msg import Int32
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, Kill
import numpy as np
import math 

class Crazzy_turtle(Node):
    def __init__(self):
        super().__init__('killer_turtle')
        self.turtle2_spawn_client = self.create_client(Spawn, "/spawn_turtle")
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.target_callback, 10)
        self.create_subscription(Pose, "/turtle2/pose", self.pose_callback, 10)
        self.create_subscription(Int32, "/finish", self.finished_callback, 10)
        self.create_timer(0.05, self.timer_callback)

        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.target_pose = np.array([0.0, 0.0, 0.0])
        self.kp_d = 3.5
        self.kp_theta = 15.0
        self.start = 0
        self.spawn_turtle()

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = 3.0
        request.y = 5.0
        request.theta = 0.0
        request.name = "turtle2"

        self.turtle2_spawn_client.call_async(request)
        
    def kill_turtle(self):
        kill = Kill.Request()
        kill.name = "turtle1"
        self.kill_turtle_client.call_async(kill)
        
    def finished_callback(self, msg):
        self.start = msg.data
        
    def pose_callback(self, msg):
        self.turtle_pose[0] = msg.x
        self.turtle_pose[1] = msg.y
        self.turtle_pose[2] = msg.theta
    
    def target_callback(self, msg):
        self.target_pose[0] = msg.x
        self.target_pose[1] = msg.y
        self.target_pose[2] = msg.theta

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def control(self, x, y):
        delta_x = ((x - self.turtle_pose[0]))
        delta_y = ((y - self.turtle_pose[1]))
        d = math.sqrt((delta_x ** 2) + (delta_y ** 2)) - 1.0

        target = math.atan2(delta_y, delta_x)
        theta = target - self.turtle_pose[2]
        w = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_d * d
        wz = self.kp_theta * w
        self.cmdvel(v, wz)

        if d < 0.1 and abs(w) < 0.1:
            self.kill_turtle()
            self.start = 0

    def timer_callback(self):
        if (self.start == 1):
            self.control(self.target_pose[0], self.target_pose[1])
        else:
            self.cmdvel(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = Crazzy_turtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
