#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- Setting & Instructions ---
msg = """
---------------------------------------
   Teleop Jog Keyboard for 3R Robot
---------------------------------------
Moving around:
   w : +X (Forward)
   s : -X (Backward)
   a : +Y (Left)
   d : -Y (Right)
   q : +Z (Up)
   e : -Z (Down)

   f : Toggle Frame (World <-> End-Effector)
   space : Stop all motion
   CTRL-C : Quit

Current Frame: {}
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_jog_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.speed = 0.2
        self.frame_mode = 0.0 # 0.0 = World, 1.0 = End-Effector
        self.frame_name = "WORLD FRAME"

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg.format(self.frame_name))

        try:
            while True:
                key = self.getKey()
                
                target_vx = 0.0
                target_vy = 0.0
                target_vz = 0.0
                
                if key == 'w':
                    target_vx = self.speed
                elif key == 's':
                    target_vx = -self.speed
                elif key == 'a':
                    target_vy = self.speed
                elif key == 'd':
                    target_vy = -self.speed
                elif key == 'q':
                    target_vz = self.speed
                elif key == 'e':
                    target_vz = -self.speed
                
                elif key == 'f':
                    if self.frame_mode == 0.0:
                        self.frame_mode = 1.0
                        self.frame_name = "END-EFFECTOR FRAME"
                    else:
                        self.frame_mode = 0.0
                        self.frame_name = "WORLD FRAME"
                    print(msg.format(self.frame_name))

                elif key == '\x03':
                    break
                
                twist = Twist()
                twist.linear.x = target_vx
                twist.linear.y = target_vy
                twist.linear.z = target_vz
                
                twist.angular.x = 0.0 
                twist.angular.y = 0.0
                twist.angular.z = self.frame_mode 

                self.publisher_.publish(twist)
                
        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()