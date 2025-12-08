#!/usr/bin/python3

from Exam1.dummy_module import dummy_function, dummy_var
import rclpy
import sys
from rclpy.node import Node
from pynput import keyboard
from geometry_msgs.msg import Twist
import math
from turtlesim.msg import Pose
from std_msgs.msg import Bool, String
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
import yaml
import os
from controller_interface.srv import SetMaxpizza, SetParam
from ament_index_python.packages import get_package_share_directory
import os

class DummyNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.declare_parameter("name_teleop", "turtle1")
        self.name_teleop = (
            self.get_parameter("name_teleop").get_parameter_value().string_value
        )

        self.declare_parameter("sampling_frequency", 100.0)
        self.rates = (
            self.get_parameter("sampling_frequency").get_parameter_value().double_value
        )

        self.shutdown_sub = self.create_subscription(String, '/shutdown_signal', self.shutdown_callback, 10)

        self.control_param = self.create_service(SetParam, self.name_teleop + "/set_param", self.set_kp_callback)
        self.max = self.create_service(SetMaxpizza, self.name_teleop + "/max_pizza", self.maxpizza_callback)

        self.finished_publish = self.create_publisher(Bool, self.name_teleop + "/finished", 10)

        #รับค่าจากkeyboard   
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        #cmd_vel
        self.turtle = self.create_publisher(Twist, self.name_teleop + self.name_teleop + '/cmd_vel',10)

        #spawn_pizza
        self.pizza = self.create_client(GivePosition, self.name_teleop + "/spawn_pizza")

        #turtlepose
        self.create_subscription(Pose, self.name_teleop + self.name_teleop + "/pose", self.turtle_pose,10)

        #eat_pizza
        self.eat = self.create_client(Empty, self.name_teleop + self.name_teleop + "/eat")

        #timmer
        self.create_timer(1.0/self.rates, self.timer_callback)

        self.kp_velo = 0.0
        self.kp_agvelo = 0.0

        self.now_pose = [0.0,0.0,0.0]
        self.list_pose = []
        self.save_pose =[]

        self.max_pizza = 0
        self.prev_maxpizza = 0
        
        self.state = 1
        self.count = 0
        self.counter = 0

        self.flag = False

        self.show_controls()

    def show_controls(self):
        self.get_logger().info("=== Control Keys ===")
        self.get_logger().info("w : เดินไปข้างหน้า")
        self.get_logger().info("s : ถอยหลัง")
        self.get_logger().info("a : หมุนซ้าย")
        self.get_logger().info("d : หมุนขวา")
        self.get_logger().info("o : spawn pizza (วางพิซซ่า)")
        self.get_logger().info("m : save เส้นทาง pizza ปัจจุบัน")
        self.get_logger().info("r : clear")

    def shutdown_callback(self, msg):
        self.get_logger().info(f"{self.get_name()} ได้รับ shutdown signal")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def maxpizza_callback(self, request:SetMaxpizza.Request, response:SetMaxpizza.Response):
        self.max_pizza = request.max_pizza.data
        if self.max_pizza >= self.prev_maxpizza:
            self.prev_maxpizza = self.max_pizza
            response.log.data = f"set pizza to {self.max_pizza}"
        else:
            response.log.data = f"set pizza error"
        return response
    
    def set_kp_callback(self,request:SetParam.Request, response:SetParam.Response ):
        self.kp_velo = request.kp_linear.data
        self.kp_agvelo = request.kp_angular.data
        return response

    def turtle_walk(self,velo ,ag_velo):
        msg = Twist()
        msg.linear.x = velo
        msg.angular.z = ag_velo
        self.turtle.publish(msg)

    def turtle_pose(self,msg):
        self.now_pose[0] = msg.x
        self.now_pose[1] = msg.y
        self.now_pose[2] = msg.theta
    
    def spawn_pizza(self,x,y):
        Pizza = GivePosition.Request()
        Pizza.x = x
        Pizza.y = y
        self.pizza.call_async(Pizza)
    
    def eater(self):
        msg = Empty.Request()
        self.eat.call_async(msg)

    def eater_calculation(self,x,y):
        axis_x = x - self.now_pose[0]
        axis_y = y - self.now_pose[1]
        distance = math.sqrt((axis_x **2)+ (axis_y **2)) - 0.1
        target = math.atan2(axis_y,axis_x)
        theta = target - self.now_pose[2]
        omega = math.atan2(math.sin(theta),math.cos(theta))
        velo = self.kp_velo * distance
        ag_velo = self.kp_agvelo * omega
        self.turtle_walk(velo,ag_velo)
            
        while self.count < len(self.list_pose):
            nx = self.list_pose[self.count][0]
            ny = self.list_pose[self.count][1]
            if math.hypot(nx - self.now_pose[0], ny - self.now_pose[1]) <= 0.3:
                self.eater()
                self.count += 1
                self.counter -= 1
            else:
                break
        return
        
    def on_press(self, key):
        if self.state == 1:
            try:
                if key.char == 'w':
                    self.turtle_walk(1.0*abs(self.kp_velo),0.0)
                elif key.char == 's':
                    self.turtle_walk(-1.0*abs(self.kp_velo) ,0.0)
                elif key.char == 'a':
                    self.turtle_walk(0.0,1.0*abs(self.kp_agvelo))
                elif key.char == 'd':
                    self.turtle_walk(0.0,-1.0*abs(self.kp_agvelo))
                elif key.char == 'o':
                    if self.counter <= self.max_pizza and len(self.save_pose) < 4:
                        self.counter += 1
                        spawn_x = self.now_pose[0]
                        spawn_y = self.now_pose[1]
                        if self.counter <= self.max_pizza:
                            self.spawn_pizza(spawn_x, spawn_y)
                        self.list_pose.append(self.now_pose.copy())
                elif key.char == 'm':
                    if len(self.save_pose) < 4:
                        if self.counter <= self.max_pizza:
                            self.save_pose.append(self.list_pose.copy())
                            self.list_pose.clear()
                        if self.counter > self.max_pizza:
                            if self.flag == False and len(self.save_pose) == 3:
                                self.save_pose.append(self.list_pose.copy())
                                self.list_pose.clear()
                                self.flag = True
                            else:
                                return
                elif key.char == "r":
                    self.state = 0
                    self.count = 0
            except AttributeError:
                pass
    
    def timer_callback(self):
        # self.get_logger().info(f"counter : {self.counter}")
        # self.get_logger().info(f"flag : {self.flag}")
        # self.get_logger().info(f"save : {len(self.save_pose)}")
        # if self.counter > self.max_pizza and len(self.save_pose) < 4:
        #     self.flag == True

        if self.state == 0:
            if self.count < len(self.list_pose):
                self.eater_calculation(self.list_pose[self.count][0], self.list_pose[self.count][1])
            else:
                self.list_pose.clear()
                self.state = 1
        if len(self.save_pose) == 4:
            package_path = get_package_share_directory('Exam1')
            save_path = os.path.join(package_path, 'pizza_paths.yaml')
            with open(save_path, "w") as f:
                # self.get_logger().info(f"Saving yaml : {self.save_pose}")
                # self.get_logger().info(f"Saving to : {save_path}")
                yaml.dump({"pizza_paths": self.save_pose}, f)
            msg = Bool()
            msg.data = True
            self.finished_publish.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()