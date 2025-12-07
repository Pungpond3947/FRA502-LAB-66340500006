#!/usr/bin/python3

from lab3.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int32
import numpy as np
from controller_interfaces.srv import SetMaxPizza, SetParam
from turtlesim.srv import Spawn, Kill
import math 

class DummyNode(Node):
    def __init__(self):
        super().__init__('eater_turtle')
        self.declare_parameter("sampling_frequency", 5.0)
        self.rates = (
            self.get_parameter("sampling_frequency").get_parameter_value().double_value
        )
        self.name = self.get_namespace()
        self.cmd_vel_pub = self.create_publisher(Twist, self.name+"/cmd_vel", 10)
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")
        self.turtle_spawn_client = self.create_client(Spawn, "/spawn_turtle")
        self.create_subscription(Pose, self.name+"/pose", self.pose_callback, 10)
        self.create_subscription(Point, "/mouse_position", self.point_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza")
        self.eat_spawn_pizza = self.create_client(Empty , self.name+"/eat")
        self.finish_pub = self.create_publisher(Int32, self.name+"/eat_status", 10)
        self.MaxPizza = self.create_service(SetMaxPizza, self.name+"/set_max_pizz", self.maxpizza_callback)
        self.control_param = self.create_service(SetParam, self.name+"/set_param", self.set_control_callback)
        self.create_timer(1.0/self.rates, self.timer_callback)
        self.create_timer(1.0, self.spawn_turtle_once)
        self.create_timer(1.0, self.kill_turtle_once)
        
        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pose = np.array([0.0,0.0])
        self.kp_d = 0.0
        self.kp_theta = 0.0
        self.count = 0
        self.pizza_count = 0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal = 0
        self.num_pizza = []
        self.max = 0
        self.finish = 0
        self.max_pizza = 0
        self.spawn = False
        self.kill = False
        
    def kill_turtle_once(self):
        if self.kill:
            return
        if not self.kill_turtle_client.service_is_ready():
            return
        self.kill_turtle()
        self.kill = True
        
    def spawn_turtle_once(self):
        if self.spawn:
            return
        if not self.turtle_spawn_client.service_is_ready():
            return
        self.spawn_turtle(self.name)
        self.spawn = True
        
    def kill_turtle(self):
        kill = Kill.Request()
        kill.name = "turtle1"
        self.kill_turtle_client.call_async(kill)
        
    def spawn_turtle(self, name):
        request = Spawn.Request()
        request.x = 5.44
        request.y = 5.44
        request.theta = 0.0
        request.name = name

        self.turtle_spawn_client.call_async(request)
        
    def spawn_pizza(self, x, y):
        pizza_pose = GivePosition.Request()
        pizza_pose.x = x
        pizza_pose.y = y
        self.spawn_pizza_client.call_async(pizza_pose)
        
    def maxpizza_callback(self, request:SetMaxPizza.Request, response:SetMaxPizza.Response):
        msg = Int32()
        msg.data = 0
        self.finish_pub.publish(msg)
        self.max_pizza = request.max_pizza.data
        response.log.data = f"set pizza to {self.max_pizza}"
        return response
    
    def set_control_callback(self, request:SetParam.Request, response:SetParam.Response):
        self.kp_d = request.kp_linear.data
        self.kp_theta = request.kp_angular.data
        return response
        
    def eat_pizza(self):
        eat = Empty.Request()
        self.eat_spawn_pizza.call_async(eat)
        
    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)
    
    def pose_callback(self, msg):
        self.turtle_pose[0] = msg.x
        self.turtle_pose[1] = msg.y
        self.turtle_pose[2] = msg.theta
    
    def point_callback(self, msg):
        self.mouse_pose[0] = msg.x
        self.mouse_pose[1] = msg.y
        if (self.max < self.max_pizza):
            self.max += 1
            self.spawn_pizza(self.mouse_pose[0], self.mouse_pose[1])
            self.pizza_count += 1
            self.num_pizza.append((self.mouse_pose[0], self.mouse_pose[1]))
        
    def goal_pose_callback(self, msg):
        self.goal = 1
        self.max += 1
        self.goal_x = msg.pose.position.x + 5.44
        self.goal_y = msg.pose.position.y + 5.44
        self.spawn_pizza(self.goal_x, self.goal_y)
        self.pizza_count += 1
        self.num_pizza.append((self.mouse_pose[0], self.mouse_pose[1]))
        
    def control(self, x, y):
        msg = Int32()
        if (self.count < self.max_pizza):
            delta_x = ((x - self.turtle_pose[0]))
            delta_y = ((y - self.turtle_pose[1]))
            d = math.sqrt(((delta_x**2) + (delta_y**2))) - 1.0
            
            pizza = math.atan2(delta_y,delta_x)
            theta = pizza - self.turtle_pose[2]
            w = math.atan2(math.sin(theta),math.cos(theta))
            
            v = self.kp_d * d
            wz = self.kp_theta * w
            self.cmdvel(v, wz)
            
            if (self.goal == 1):
                if (d < 0.5 and abs(w) < 0.1):
                    self.eat_pizza()
                    self.goal = 0
                    self.count += 1
            else:
                if (d < 0.5 and abs(w) < 0.1):
                    self.eat_pizza()
                    self.count += 1
        elif (self.count != 0 and self.count >= self.max_pizza):
            msg.data = 1
            self.finish_pub.publish(msg)
            delta_x = ((x - self.turtle_pose[0]))
            delta_y = ((y - self.turtle_pose[1]))
            d = math.sqrt(((delta_x**2) + (delta_y**2))) - 1.0
            
            target = math.atan2(delta_y,delta_x)
            theta = target - self.turtle_pose[2]
            w = math.atan2(math.sin(theta),math.cos(theta))
            
            v = self.kp_d * d
            wz = self.kp_theta * w
            self.cmdvel(v, wz)
            
    def timer_callback(self):
        if (self.count < self.max_pizza):
            if (self.goal == 1):
                self.control(self.goal_x, self.goal_y)
            else:
                if (self.count < self.pizza_count):
                    if len(self.num_pizza) == 0:
                        return
                    else:
                        self.control(self.num_pizza[self.count][0], self.num_pizza[self.count][1])
                elif (self.count == self.pizza_count):
                    self.cmdvel(0.0,0.0)
        else:
            self.control(self.mouse_pose[0], self.mouse_pose[1])

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
