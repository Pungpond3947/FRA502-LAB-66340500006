#!/usr/bin/python3

from lab3.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from turtlesim.srv import Spawn, Kill
from controller_interfaces.srv import SetParam
import numpy as np
import math 

class Crazzy_turtle(Node):
    def __init__(self):
        super().__init__('killer_turtle')
        self.declare_parameter("sampling_frequency", 5.0)
        self.rates = (
            self.get_parameter("sampling_frequency").get_parameter_value().double_value
        )
        self.declare_parameter("name1")
        self.name1 = (
            self.get_parameter("name1").get_parameter_value().string_value
        )
        self.name2 = self.get_namespace()
        self.turtle_spawn_client = self.create_client(Spawn, "/spawn_turtle")
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")
        self.control_param = self.create_service(SetParam, self.name2+"/set_param", self.set_control_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, self.name2+"/cmd_vel", 10)
        self.create_subscription(Pose, self.name1+"/pose", self.target_callback, 10)
        self.create_subscription(Pose, self.name2+"/pose", self.pose_callback, 10)
        self.create_subscription(Int32, self.name1+"/eat_status", self.finished_callback, 10)
        self.create_timer(1.0/self.rates, self.timer_callback)
        self.create_timer(1.0, self.spawn_turtle_once)

        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.target_pose = np.array([0.0, 0.0, 0.0])
        self.kp_d = 0.0
        self.kp_theta = 0.0
        self.start = 0
        self.spawn = False
        
    def spawn_turtle_once(self):
        if self.spawn:
            return
        if not self.turtle_spawn_client.service_is_ready():
            return
        self.spawn_turtle(self.name2)
        self.spawn = True
        
    def set_control_callback(self, request:SetParam.Request, response:SetParam.Response):
        self.kp_d = request.kp_linear.data
        self.kp_theta = request.kp_angular.data
        return response

    def spawn_turtle(self, name):
        request = Spawn.Request()
        request.x = 1.0
        request.y = 1.0
        request.theta = 0.0
        request.name = name

        self.turtle_spawn_client.call_async(request)
        
    def kill_turtle(self):
        kill = Kill.Request()
        kill.name = self.name1
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
