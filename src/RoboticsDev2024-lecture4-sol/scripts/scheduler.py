#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from lab4_interfaces.srv import SetMode, Random
from geometry_msgs.msg import Pose

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')

        self.cb_group = ReentrantCallbackGroup()

        self.mode_service = self.create_service(
            SetMode, 
            'set_mode', 
            self.set_mode_callback, 
            callback_group=self.cb_group
        )
        
        self.ctrl_client = self.create_client(SetMode, 'robot_command', callback_group=self.cb_group)
        self.rand_client = self.create_client(Random, 'random_server', callback_group=self.cb_group)

        self.current_mode = "IDLE"
        self.auto_timer = None

        self.get_logger().info("Scheduler (Manager) Ready [Multi-threaded].")

    def set_mode_callback(self, request, response):
        self.current_mode = request.mode
        self.get_logger().info(f"Manager: Switching to {self.current_mode}")

        if self.auto_timer:
            self.auto_timer.cancel()
            self.auto_timer = None

        if request.mode == "IPK":
            result = self.send_command_to_controller("IPK", request.target_pose)
            response.success = result.success
            response.message = "IPK Mode Started"
            response.configuration = result.configuration
            return response

        elif request.mode == "TELEOP":
            result = self.send_command_to_controller("TELEOP", Pose())
            response.success = result.success
            response.message = "Teleop Mode Started"
            return response

        elif request.mode == "AUTO":
            self.trigger_auto_sequence()
            response.success = True
            response.message = "Auto Mode Started"
            return response
        
        else:
            response.success = False
            response.message = "Unknown Mode"
            return response

    def send_command_to_controller(self, mode, target):
        if not self.ctrl_client.wait_for_service(timeout_sec=1.0):
            res = SetMode.Response()
            res.success = False
            res.message = "Controller unavailable"
            return res

        req = SetMode.Request()
        req.mode = mode
        req.target_pose = target
        
        return self.ctrl_client.call(req)

    def trigger_auto_sequence(self):
        if self.current_mode != "AUTO": return

        if not self.rand_client.wait_for_service(1.0):
            return
            
        req = Random.Request()
        req.mode = "AUTO"
        
        future = self.rand_client.call_async(req)
        future.add_done_callback(self.on_random_received)

    def on_random_received(self, future):
        if self.current_mode != "AUTO": return
        try:
            res = future.result()
            target = Pose()
            target.position.x = res.position.x
            target.position.y = res.position.y
            target.position.z = res.position.z

            self.get_logger().info(f"Scheduler : Got Random Target. Sending to Controller...")
            self.send_command_to_controller("IPK", target)

            # รอ 10 วินาทีแล้ววนใหม่
            self.auto_timer = self.create_timer(10.0, self.auto_timer_callback, callback_group=self.cb_group)

        except Exception as e:
            self.get_logger().error(f"Auto Error: {e}")

    def auto_timer_callback(self):
        self.auto_timer.cancel()
        self.trigger_auto_sequence()

def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    
    # ใช้ MultiThreadedExecutor เพื่อให้ callback ซ้อนกันได้
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()