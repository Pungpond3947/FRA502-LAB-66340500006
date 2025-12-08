#!/usr/bin/python3

from Exam1.dummy_module import dummy_function, dummy_var
import rclpy
import sys
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from controller_interface.srv import SetParam
from turtlesim_plus_interfaces.srv import GivePosition
import numpy as np
import math
import yaml
from ament_index_python.packages import get_package_share_directory
import os


class DummyNode(Node):
    def __init__(self):
        super().__init__('copy_turtle_node')

        self.declare_parameter("name_teleop", "teleop")
        self.name_teleop = (
            self.get_parameter("name_teleop").get_parameter_value().string_value
        )
        self.declare_parameter("name_copy", "copy")
        self.name_copy = (
            self.get_parameter("name_copy").get_parameter_value().string_value
        )
        self.declare_parameter("turtle1", "Foxy")
        self.name1 = (
            self.get_parameter("turtle1").get_parameter_value().string_value
        )
        self.declare_parameter("turtle2", "Noetic")
        self.name2 = (
            self.get_parameter("turtle2").get_parameter_value().string_value
        )
        self.declare_parameter("turtle3", "Humble")
        self.name3 = (
            self.get_parameter("turtle3").get_parameter_value().string_value
        )
        self.declare_parameter("turtle4", "Iron")
        self.name4 = (
            self.get_parameter("turtle4").get_parameter_value().string_value
        )

        self.declare_parameter("sampling_frequency", 100.0)
        self.rates = (
            self.get_parameter("sampling_frequency").get_parameter_value().double_value
        )

        self.shutdown_sub = self.create_subscription(String, '/shutdown_signal', self.shutdown_callback, 10)

        self.control_param = self.create_service(SetParam, self.name_copy + "/set_param", self.set_control_callback)
        # self.turtle1_eatpizza = self.create_client(Empty , self.name_copy + "/" + self.name1 + "/eat")
        # self.turtle2_eatpizza = self.create_client(Empty , self.name_copy + "/" + self.name2 + "/eat")
        # self.turtle3_eatpizza = self.create_client(Empty , self.name_copy + "/" + self.name3 + "/eat")
        # self.turtle4_eatpizza = self.create_client(Empty , self.name_copy + "/" + self.name4 + "/eat")

        self.spawn_pizza_client = self.create_client(GivePosition, self.name_copy + "/spawn_pizza")

        self.create_subscription(Pose, self.name_copy + "/" + self.name1 + "/pose", self.pose1_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name2 + "/pose", self.pose2_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name3 + "/pose", self.pose3_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name4 + "/pose", self.pose4_callback, 10)

        self.create_subscription(Bool, self.name_teleop + "/finished", self.finished_callback, 10)

        self.create_subscription(Point, self.name_copy + "/mouse_position", self.mouse_callback, 10)

        self.cmdvel1_publish = self.create_publisher(Twist, self.name_copy + "/" + self.name1 + "/cmd_vel", 10)
        self.cmdvel2_publish = self.create_publisher(Twist, self.name_copy + "/" + self.name2 + "/cmd_vel", 10)
        self.cmdvel3_publish = self.create_publisher(Twist, self.name_copy + "/" + self.name3 + "/cmd_vel", 10)
        self.cmdvel4_publish = self.create_publisher(Twist, self.name_copy + "/" + self.name4 + "/cmd_vel", 10)

        self.mouse_publish = self.create_publisher(Point, self.name_copy + "/potal", 10)
        self.finish_publish = self.create_publisher(Bool, self.name_copy + "/finish", 10)

        self.turtle1_pose = np.array([0.0, 0.0, 0.0])
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])
        self.turtle3_pose = np.array([0.0, 0.0, 0.0])
        self.turtle4_pose = np.array([0.0, 0.0, 0.0])

        self.kp_d = 0.0
        self.kp_theta = 0.0

        self.mouse_x = 0.0
        self.mouse_y = 0.0
        self.count_mouse = 0

        self.move1 = False
        self.move2 = False
        self.move3 = False
        self.move4 = False
        self.finish = False
        self.can_make_potal = False

        self.next1 = 0
        self.next2 = 0
        self.next3 = 0
        self.next4 = 0

        self.paths = []
        self.paths1_count = 0
        self.paths2_count = 0
        self.paths3_count = 0
        self.paths4_count = 0

        self.create_timer(1.0/self.rates, self.timer_callback)

    def shutdown_callback(self, msg):
        self.get_logger().info(f"{self.get_name()} ได้รับ shutdown signal")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    def set_control_callback(self, request:SetParam.Request, response:SetParam.Response):
        self.kp_d = request.kp_linear.data
        self.kp_theta = request.kp_angular.data
        return response
    
    def finished_callback(self, msg):
        package_path = get_package_share_directory('Exam1')
        load_path = os.path.join(package_path, 'pizza_paths.yaml')
        try:
            with open(load_path, "r") as f:
                data = yaml.safe_load(f)

            # ถ้าไฟล์ว่างหรือไม่มีข้อมูล → data จะเป็น None
            if not data or "pizza_paths" not in data:
                self.get_logger().error("pizza_paths not found in yaml or file is empty")
                self.paths = [[], [], [], []]
                self.move1 = False
                self.move2 = False
                self.move3 = False
                self.move4 = False
                return

            pizza_paths = data["pizza_paths"]
            if not isinstance(pizza_paths, list):
                self.get_logger().error("pizza_paths is not a list")
                self.paths = [[], [], [], []]
                self.move1 = False
                self.move2 = False
                self.move3 = False
                self.move4 = False
                return

            # ถ้า path มีน้อยกว่า 4 list → เติม [] ให้ครบ
            if len(pizza_paths) < 4:
                pizza_paths = (pizza_paths + [[] , [] , [] , []])[:4]

            # บังคับให้ทุก element เป็น list
            pizza_paths = [p if isinstance(p, list) else [] for p in pizza_paths]

            self.paths = pizza_paths

            self.move1 = msg.data
            self.move2 = msg.data
            self.move3 = msg.data
            self.move4 = msg.data
            # self.get_logger().info(f"Pizza paths loaded: {len(self.paths)} groups")
            
        except Exception as e:
            # self.get_logger().error(f"Failed to load pizza_paths: {e}")
            self.paths = [[], [], [], []]
            self.move1 = False
            self.move2 = False
            self.move3 = False
            self.move4 = False

    def pose1_callback(self, msg):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y
        self.turtle1_pose[2] = msg.theta

    def pose2_callback(self, msg):
        self.turtle2_pose[0] = msg.x
        self.turtle2_pose[1] = msg.y
        self.turtle2_pose[2] = msg.theta

    def pose3_callback(self, msg):
        self.turtle3_pose[0] = msg.x
        self.turtle3_pose[1] = msg.y
        self.turtle3_pose[2] = msg.theta

    def pose4_callback(self, msg):
        self.turtle4_pose[0] = msg.x
        self.turtle4_pose[1] = msg.y
        self.turtle4_pose[2] = msg.theta

    def mouse_callback(self, msg):
        if self.finish == True:
            self.mouse_x = msg.x
            self.mouse_y = msg.y
            self.count_mouse += 1

            if self.count_mouse == 1:
                self.can_make_potal = True
                mouse_pose = Point()
                mouse_pose.x = self.mouse_x
                mouse_pose.y = self.mouse_y
                self.mouse_publish.publish(mouse_pose)

    def eat_pizza(self, name):
        eat = Empty.Request()
        if name == self.name1:
            self.turtle1_eatpizza.call_async(eat)
        elif name == self.name2:
            self.turtle2_eatpizza.call_async(eat)
        elif name == self.name3:
            self.turtle3_eatpizza.call_async(eat)
        elif name == self.name4:
            self.turtle4_eatpizza.call_async(eat)

    def spawn_pizza(self, x, y):
        pizza_pose = GivePosition.Request()
        pizza_pose.x = x
        pizza_pose.y = y
        self.spawn_pizza_client.call_async(pizza_pose)

    def cmdvel(self, v, w, name):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        if name == self.name1:
            self.cmdvel1_publish.publish(msg)
        elif name == self.name2:
            self.cmdvel2_publish.publish(msg)
        elif name == self.name3:
            self.cmdvel3_publish.publish(msg)
        elif name == self.name4:
            self.cmdvel4_publish.publish(msg)

    def control(self, x, y, name):
        if name == self.name1:
            x_turtle = self.turtle1_pose[0]
            y_turtle = self.turtle1_pose[1]
            theta_turtle = self.turtle1_pose[2]
        elif name == self.name2:
            x_turtle = self.turtle2_pose[0]
            y_turtle = self.turtle2_pose[1]
            theta_turtle = self.turtle2_pose[2]
        elif name == self.name3:
            x_turtle = self.turtle3_pose[0]
            y_turtle = self.turtle3_pose[1]
            theta_turtle = self.turtle3_pose[2]
        elif name == self.name4:
            x_turtle = self.turtle4_pose[0]
            y_turtle = self.turtle4_pose[1]
            theta_turtle = self.turtle4_pose[2]

        delta_x = ((x - x_turtle))
        delta_y = ((y - y_turtle))
        d = math.sqrt((delta_x ** 2) + (delta_y ** 2))

        target = math.atan2(delta_y, delta_x)
        theta = target - theta_turtle
        w = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_d * d
        wz = self.kp_theta * w
        self.cmdvel(v, wz, name)

        if d < 0.1 and abs(w) < 0.1:
            if name == self.name1:
                if self.paths1_count < len(self.paths[0]):
                    self.spawn_pizza(self.paths[0][self.paths1_count][0], self.paths[0][self.paths1_count][1])
                    self.paths1_count += 1
                    # self.get_logger().info(f"count1 = {self.paths1_count}")
            elif name == self.name2:
                if self.paths2_count < len(self.paths[1]):
                    self.spawn_pizza(self.paths[1][self.paths2_count][0], self.paths[1][self.paths2_count][1])
                    self.paths2_count += 1
                    # self.get_logger().info(f"count2 = {self.paths2_count}")
            elif name == self.name3:
                if self.paths3_count < len(self.paths[2]):
                    self.spawn_pizza(self.paths[2][self.paths3_count][0], self.paths[2][self.paths3_count][1])
                    self.paths3_count += 1
                    # self.get_logger().info(f"count3 = {self.paths3_count}")
            elif name == self.name4:
                if self.paths4_count < len(self.paths[3]):
                    self.spawn_pizza(self.paths[3][self.paths4_count][0], self.paths[3][self.paths4_count][1])
                    self.paths4_count += 1
                    # self.get_logger().info(f"count4 = {self.paths4_count}")

    def timer_callback(self):
        if self.move1 == True:
            if self.paths1_count < len(self.paths[0]):
                self.control(self.paths[0][self.paths1_count][0], self.paths[0][self.paths1_count][1], self.name1)
            else:
                self.move1 = False
                self.next1 = 1
                # self.get_logger().info(f"move1 = {self.move1}")
                self.cmdvel(0.0, 0.0, self.name1)
        if self.move2 == True:
            if self.paths2_count < len(self.paths[1]):
                self.control(self.paths[1][self.paths2_count][0], self.paths[1][self.paths2_count][1], self.name2)
            else:
                self.move2 = False
                self.next2 = 1
                # self.get_logger().info(f"move2 = {self.move2}")
                self.cmdvel(0.0, 0.0, self.name2)
        if self.move3 == True:
            if self.paths3_count < len(self.paths[2]):
                self.control(self.paths[2][self.paths3_count][0], self.paths[2][self.paths3_count][1], self.name3)
            else:
                self.move3 = False
                self.next3 = 1
                # self.get_logger().info(f"move3 = {self.move3}")
                self.cmdvel(0.0, 0.0, self.name3)
        if self.move4 == True:
            if self.paths4_count < len(self.paths[3]):
                self.control(self.paths[3][self.paths4_count][0], self.paths[3][self.paths4_count][1], self.name4)
            else:
                self.move4 = False
                self.next4 = 1
                # self.get_logger().info(f"move4 = {self.move4}")
                self.cmdvel(0.0, 0.0, self.name4)
                
        if self.next1 == 1 and self.next2 == 1 and self.next3 == 1 and self.next4 == 1:
            if self.paths1_count >= len(self.paths[0]) and self.paths2_count >= len(self.paths[1]) and self.paths3_count >= len(self.paths[2]) and self.paths4_count >= len(self.paths[3]):
                self.finish = True
                # self.get_logger().info(f"finish = {self.finish}")
                msg = Bool()
                msg.data = True
                self.finish_publish.publish(msg)

        if self.can_make_potal == True:
            self.control(self.mouse_x, self.mouse_y, self.name1)
            self.control(self.mouse_x, self.mouse_y, self.name2)
            self.control(self.mouse_x, self.mouse_y, self.name3)
            self.control(self.mouse_x, self.mouse_y, self.name4)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()