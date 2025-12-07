#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int32
import numpy as np
import math 

class DummyNode(Node):
    def __init__(self):
        super().__init__('eater_turtle')
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.create_subscription(Point, "/mouse_position", self.point_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza")
        self.eat_spawn_pizza = self.create_client(Empty , "/turtle1/eat")
        self.finish_pub = self.create_publisher(Int32, "/finish", 10)
        self.create_timer(0.05, self.timer_callback)
        
        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pose = np.array([0.0,0.0])
        self.kp_d = 5.0
        self.kp_theta = 15.0
        self.count = 0
        self.pizza_count = 0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal = 0
        self.num_pizza = []
        self.max = 0
        self.finish = 0
        
    def spawn_pizza(self, x, y):
        pizza_pose = GivePosition.Request()
        pizza_pose.x = x
        pizza_pose.y = y
        self.spawn_pizza_client.call_async(pizza_pose)
        
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
        self.max += 1
        if (self.max <= 20):
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
        if (self.count < 20):
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
        else:
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
        if (self.count < 20):
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
