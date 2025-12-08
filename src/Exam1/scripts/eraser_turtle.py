#!/usr/bin/python3

from Exam1.dummy_module import dummy_function, dummy_var
import rclpy
import sys
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from std_msgs.msg import Bool, String
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Point
from controller_interface.srv import SetParam
import numpy as np
import math
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class DummyNode(Node):
    def __init__(self):
        super().__init__('eraser_node')
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
        self.declare_parameter("name_eraser", "eraser")
        self.name_eraser = (
            self.get_parameter("name_eraser").get_parameter_value().string_value
        )

        self.declare_parameter("sampling_frequency", 100.0)
        self.rates = (
            self.get_parameter("sampling_frequency").get_parameter_value().double_value
        )

        self.status_pub = self.create_publisher(String, '/status', 10)
        self.shutdown_sub = self.create_subscription(String, '/shutdown_signal', self.shutdown_callback, 10)

        self.control_param = self.create_service(SetParam, self.name_eraser + "/set_param", self.set_control_callback)

        self.create_subscription(Point, self.name_copy + "/potal", self.potal_callback, 10)
        self.create_subscription(Bool, self.name_copy + "/finish", self.copy_finished_callback, 10)
        self.create_subscription(Pose, self.name_teleop + "/" + self.name_eraser + "/pose", self.eraserteleop_pose_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name_eraser + "/pose", self.erasercopy_pose_callback, 10)
        self.create_subscription(Pose, self.name_teleop + self.name_teleop + "/pose", self.teleop_pose_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name1 + "/pose", self.turtle1_pose_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name2 + "/pose", self.turtle2_pose_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name3 + "/pose", self.turtle3_pose_callback, 10)
        self.create_subscription(Pose, self.name_copy + "/" + self.name4 + "/pose", self.turtle4_pose_callback, 10)

        self.cmdvel1_publish = self.create_publisher(Twist, self.name_teleop + "/" + self.name_eraser + "/cmd_vel", 10)
        self.cmdvel2_publish = self.create_publisher(Twist, self.name_copy + "/" + self.name_eraser + "/cmd_vel", 10)

        self.spawn_turtle1_client = self.create_client(Spawn, self.name_teleop + "/spawn_turtle")
        self.spawn_turtle2_client = self.create_client(Spawn, self.name_copy + "/spawn_turtle")
        self.eat1_pizza_client = self.create_client(Empty , self.name_teleop + "/" + self.name_eraser + "/eat")
        self.eat2_pizza_client = self.create_client(Empty , self.name_copy + "/" + self.name_eraser + "/eat")
        self.kill_turtle1_client = self.create_client(Kill, self.name_teleop + "/remove_turtle")
        self.kill_turtle2_client = self.create_client(Kill, self.name_copy + "/remove_turtle")

        self.potal_pose = np.array([0.0, 0.0])
        self.eraserteleop_pose = np.array([0.0, 0.0, 0.0])
        self.erasercopy_pose = np.array([0.0, 0.0, 0.0])
        self.teleop_pose = np.array([0.0, 0.0, 0.0])
        self.turtle1_pose = np.array([0.0, 0.0])
        self.turtle2_pose = np.array([0.0, 0.0])
        self.turtle3_pose = np.array([0.0, 0.0])
        self.turtle4_pose = np.array([0.0, 0.0])

        self.paths = []
        self.path1_count = 0
        self.path2_count = 0

        self.state = 0

        self.teleop_paths_finished = False

        self.eraser_spawned_teleop = False
        self.eraser_spawned_copy = False
        self.eraser_killed_teleop = False
        self.eraser_killed_eraser = False
        self.eraser_killed_Foxy = False
        self.eraser_killed_Noetic = False
        self.eraser_killed_Humble = False
        self.eraser_killed_Iron = False
        self.erasercopy_killed_erasercopy = False

        self.kp_d = 0.0
        self.kp_theta = 0.0

        self.have_potal = False

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
    
    def eat_pizza(self, namespace):
        eat = Empty.Request()
        if namespace == self.name_teleop:
            self.eat1_pizza_client.call_async(eat)
        elif namespace == self.name_copy:
            self.eat2_pizza_client.call_async(eat)

    def kill_turtle(self, name, namespace):
        kill = Kill.Request()
        kill.name = name
        
        if namespace == self.name_teleop:
            if name == namespace:
                name = name[1:]
                kill.name = name
            client = self.kill_turtle1_client
        elif namespace == self.name_copy:
            client = self.kill_turtle2_client

        future = client.call_async(kill)
        return future

    def potal_callback(self, msg):
        self.potal_pose[0] = msg.x
        self.potal_pose[1] = msg.y

        self.have_potal = True

    def copy_finished_callback(self, msg):
        package_path = get_package_share_directory('Exam1')
        load_path = os.path.join(package_path, 'pizza_paths.yaml')

        if self.paths:
            return

        if not os.path.exists(load_path) or os.path.getsize(load_path) == 0:
            default_data = {
                "pizza_paths": [
                    [[0.5, 1.0], [1.0, 2.0]],
                    [[1.5, 1.0], [2.0, 2.0]],
                    [[0.5, 2.0], [1.5, 2.5]],
                    [[1.0, 1.0], [2.0, 1.5]],
                ]
            }
            with open(load_path, "w") as f:
                yaml.dump(default_data, f)
            self.get_logger().info(f"Created default pizza_paths in {load_path}")

        try:
            with open(load_path, "r") as f:
                data = yaml.safe_load(f) or {}

            pizza_paths = data.get("pizza_paths", [])

            while len(pizza_paths) < 4:
                pizza_paths.append([])

            pizza_paths = [p if isinstance(p, list) else [] for p in pizza_paths]

            self.paths = pizza_paths
            self.state = 1
            self.get_logger().info(f"Pizza paths loaded successfully: {[len(p) for p in self.paths]} points per group")

        except Exception as e:
            self.get_logger().error(f"Failed to load pizza_paths: {e}")
            self.paths = [[], [], [], []]
            self.state = 1

    def eraserteleop_pose_callback(self, msg):
        self.eraserteleop_pose[0] = msg.x
        self.eraserteleop_pose[1] = msg.y
        self.eraserteleop_pose[2] = msg.theta

    def erasercopy_pose_callback(self, msg):
        self.erasercopy_pose[0] = msg.x
        self.erasercopy_pose[1] = msg.y
        self.erasercopy_pose[2] = msg.theta

    def teleop_pose_callback(self, msg):
        self.teleop_pose[0] = msg.x
        self.teleop_pose[1] = msg.y
        self.teleop_pose[2] = msg.theta

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y

    def turtle2_pose_callback(self, msg):
        self.turtle2_pose[0] = msg.x
        self.turtle2_pose[1] = msg.y

    def turtle3_pose_callback(self, msg):
        self.turtle3_pose[0] = msg.x
        self.turtle3_pose[1] = msg.y

    def turtle4_pose_callback(self, msg):
        self.turtle4_pose[0] = msg.x
        self.turtle4_pose[1] = msg.y

    def spawn_turtle(self, x, y, name, namespace):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = name

        if namespace == self.name_teleop:
            client = self.spawn_turtle1_client
        elif namespace == self.name_copy:
            client = self.spawn_turtle2_client

        future = client.call_async(request)
        return future

    def cmdvel(self, v, w, namespace):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        if namespace == self.name_teleop:
            self.cmdvel1_publish.publish(msg)
        elif namespace == self.name_copy:
            self.cmdvel2_publish.publish(msg)

    def control(self, x, y, pose, namespace):
        delta_x = ((x - pose[0]))
        delta_y = ((y - pose[1]))
        d = math.sqrt((delta_x ** 2) + (delta_y ** 2))

        target = math.atan2(delta_y, delta_x)
        theta = target - pose[2]
        w = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_d * d
        wz = self.kp_theta * w
        self.cmdvel(v, wz, namespace)

        if d < 0.1 and abs(w) < 0.1:
            if self.state == 1 or self.state == 14:
                
                while self.path1_count < len(self.paths) and not self.paths[self.path1_count]:
                    self.path1_count += 1
                    self.path2_count = 0

                if self.path1_count >= len(self.paths):
                    self.path1_count = 0
                    self.path2_count = 0
                    if self.state == 1:
                        self.state = 2
                    elif self.state == 14:
                        self.state = 15
                    return
                
                if self.state == 1:
                    self.eat_pizza(self.name_teleop)
                elif self.state == 14:
                    self.eat_pizza(self.name_copy)

                self.path2_count += 1

                if self.path2_count >= len(self.paths[self.path1_count]):
                    self.path2_count = 0
                    self.path1_count += 1

                    while self.path1_count < len(self.paths) and not self.paths[self.path1_count]:
                        self.path1_count += 1
                        self.path2_count = 0

                    if self.path1_count >= len(self.paths):
                        self.path1_count = 0
                        self.path2_count = 0
                        if self.state == 1:
                            self.state = 2
                        elif self.state == 14:
                            self.state = 15
                        return

            elif self.state == 2:
                self.state = 3
                return
            elif self.state == 5:
                self.state = 6
                return
            elif self.state == 6:
                self.state = 7
                return
            elif self.state == 8:
                self.state = 9
                return
            elif self.state == 10:
                self.state = 11
                return
            elif self.state == 12:
                self.state = 13
                return

    def timer_callback(self):
        # self.get_logger().info(f"state = {self.state}")

        if self.state == 1:
            if (not self.paths or self.path1_count >= len(self.paths)):
                # ถ้า path โหลดมาแล้วว่างเปล่าทั้งหมดตั้งแต่ต้น หรือ index เกิน ให้ข้าม
                if not self.paths or (self.paths and all(not p for p in self.paths)):
                     self.get_logger().warn("No paths or all paths empty, skipping State 1")
                     self.state = 2
                     return
            
            if not self.eraser_spawned_teleop:
                def spawn_done(fut):
                    try:
                        fut.result()
                        self.get_logger().info(f"{self.name_eraser} spawned in {self.name_teleop}")
                    except Exception as e:
                        self.get_logger().error(f"Failed to spawn {self.name_eraser} in {self.name_teleop}: {e}")

                if self.have_potal == True:
                    future = self.spawn_turtle(0.1, 0.1, self.name_eraser, self.name_teleop)
                    future.add_done_callback(spawn_done)
                    self.eraser_spawned_teleop = True

            # รอ spawn เสร็จก่อน และมั่นใจว่ามี path
            if self.eraser_spawned_teleop and self.paths:
                while self.path1_count < len(self.paths) and not self.paths[self.path1_count]:
                    self.path1_count += 1
                    self.path2_count = 0
                
                # เช็คซ้ำว่าหลังจากวน loop แล้ว index ยังอยู่ในขอบเขตไหม
                if self.path1_count >= len(self.paths):
                    self.path1_count = 0
                    self.path2_count = 0
                    self.state = 2
                    return

                self.control(self.paths[self.path1_count][self.path2_count][0],self.paths[self.path1_count][self.path2_count][1], self.eraserteleop_pose, self.name_teleop)

        elif self.state == 2:
            self.control(self.teleop_pose[0], self.teleop_pose[1], self.eraserteleop_pose, self.name_teleop)

        elif self.state == 3:
            if not self.eraser_killed_teleop:
                future = self.kill_turtle(self.name_teleop, self.name_teleop)
                if future:
                    def kill_done(fut):
                        try:
                            fut.result()
                            self.get_logger().info(f"{self.name_teleop} killed")
                            self.state = 4 
                        except Exception as e:
                            self.get_logger().error(f"Failed to kill {self.name_teleop}: {e}")

                    future.add_done_callback(kill_done)
                    self.eraser_killed_teleop = True

        elif self.state == 4:
            if not self.eraser_killed_eraser:
                future = self.kill_turtle(self.name_eraser, self.name_teleop)
                if future:
                    def kill_done(fut):
                        try:
                            fut.result()
                            self.get_logger().info(f"{self.name_eraser} killed in {self.name_eraser}")
                            self.state = 5
                        except Exception as e:
                            self.get_logger().error(f"Failed to kill {self.name_eraser}: {e}")

                    future.add_done_callback(kill_done)
                    self.eraser_killed_eraser = True

        elif self.state == 5 and self.potal_pose[0] > 0.0:
            if not self.eraser_spawned_copy:
                def spawn_done(fut):
                    try:
                        fut.result()
                        self.get_logger().info(f"{self.name_eraser} spawned in {self.name_copy}")
                        self.state = 6 
                    except Exception as e:
                        self.get_logger().error(f"Failed to spawn {self.name_eraser} in {self.name_copy}: {e}")

                future = self.spawn_turtle(self.potal_pose[0], self.potal_pose[1], self.name_eraser, self.name_copy)
                future.add_done_callback(spawn_done)
                self.eraser_spawned_copy = True

        elif self.state == 6:
            self.control(self.turtle1_pose[0], self.turtle1_pose[1], self.erasercopy_pose, self.name_copy)

        elif self.state == 7:
            if not self.eraser_killed_Foxy:
                future = self.kill_turtle(self.name1, self.name_copy)
                if future:
                    def kill_done(fut):
                        try:
                            fut.result()
                            self.get_logger().info(f"{self.name_eraser} killed in {self.name_copy}")
                            self.state = 8
                        except Exception as e:
                            self.get_logger().error(f"Failed to kill {self.name1}: {e}")

                    future.add_done_callback(kill_done)
                    self.eraser_killed_Foxy = True
        
        elif self.state == 8:
            self.control(self.turtle2_pose[0], self.turtle2_pose[1], self.erasercopy_pose, self.name_copy)

        elif self.state == 9:
            if not self.eraser_killed_Noetic:
                future = self.kill_turtle(self.name2, self.name_copy)
                if future:
                    def kill_done(fut):
                        try:
                            fut.result()
                            self.get_logger().info(f"{self.name_eraser} killed in {self.name_copy}")
                            self.state = 10
                        except Exception as e:
                            self.get_logger().error(f"Failed to kill {self.name2}: {e}")

                    future.add_done_callback(kill_done)
                    self.eraser_killed_Noetic = True

        elif self.state == 10:
            self.control(self.turtle3_pose[0], self.turtle3_pose[1], self.erasercopy_pose, self.name_copy)

        elif self.state == 11:
            if not self.eraser_killed_Humble:
                future = self.kill_turtle(self.name3, self.name_copy)
                if future:
                    def kill_done(fut):
                        try:
                            fut.result()
                            self.get_logger().info(f"{self.name_eraser} killed in {self.name_copy}")
                            self.state = 12
                        except Exception as e:
                            self.get_logger().error(f"Failed to kill {self.name3}: {e}")

                    future.add_done_callback(kill_done)
                    self.eraser_killed_Humble = True
        
        elif self.state == 12:
            self.control(self.turtle4_pose[0], self.turtle4_pose[1], self.erasercopy_pose, self.name_copy)

        elif self.state == 13:
            if not self.eraser_killed_Iron:
                future = self.kill_turtle(self.name4, self.name_copy)
                if future:
                    def kill_done(fut):
                        try:
                            fut.result()
                            self.get_logger().info(f"{self.name_eraser} killed in {self.name_copy}")
                            self.state = 14
                        except Exception as e:
                            self.get_logger().error(f"Failed to kill {self.name4}: {e}")

                    future.add_done_callback(kill_done)
                    self.eraser_killed_Iron = True
        
        elif self.state == 14:
            # แก้ไข: เพิ่มการเช็ค path ว่างก่อนส่งไป control
            while self.path1_count < len(self.paths) and not self.paths[self.path1_count]:
                self.path1_count += 1
                self.path2_count = 0
            
            # ถ้าวนแล้วพบว่าว่างหมด (index เกิน) ให้ไป state ถัดไปเลย
            if self.path1_count >= len(self.paths):
                self.path1_count = 0
                self.path2_count = 0
                self.state = 15
                return

            self.control(self.paths[self.path1_count][self.path2_count][0],self.paths[self.path1_count][self.path2_count][1], self.erasercopy_pose, self.name_copy)

        elif self.state == 15:
            if not self.erasercopy_killed_erasercopy:
                future = self.kill_turtle(self.name_eraser, self.name_copy)
                if future:
                    def kill_done(fut):
                        try:
                            fut.result()
                            self.get_logger().info(f"{self.name_eraser} killed in {self.name_eraser}")
                            self.state = 16
                        except Exception as e:
                            self.get_logger().error(f"Failed to kill {self.name_eraser}: {e}")

                    future.add_done_callback(kill_done)
                    self.erasercopy_killed_erasercopy = True

        elif self.state == 16:
            nodes = ['teleop_node', 'kill_node', 'spawn_node', 'eraser_node', 'copy_turtle_node']
            for n in nodes:
                msg = String()
                msg.data = n
                self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()