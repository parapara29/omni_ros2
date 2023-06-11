#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class OmniDriveController(Node):
    def __init__(self):
        super().__init__('omni_drive_controller')

        # Wheel controller publishers
        self.wheel_vel = np.array([0,0,0,0], float)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        timer_period = 0.005
        self.L = 0.125 # distance from the robot center to wheel
        self.Rw = 0.03 # Radius ot the wheel

        self.vel_msg = Twist()
        self.threshold = 0.08
        self.linear_vel = 1
        self.omega = 3

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global axes

        vel_x = axes[0]*self.linear_vel
        vel_y = axes[1]*self.linear_vel
        vel_w = axes[2]*self.omega

        self.wheel_vel[0] = (vel_x*math.sin(math.pi/4            ) + vel_y*math.cos(math.pi/4            ) + self.L*vel_w)/self.Rw
        self.wheel_vel[1] = (vel_x*math.sin(math.pi/4 + math.pi/2) + vel_y*math.cos(math.pi/4 + math.pi/2) + self.L*vel_w)/self.Rw
        self.wheel_vel[2] = (vel_x*math.sin(math.pi/4 - math.pi)   + vel_y*math.cos(math.pi/4 - math.pi)   + self.L*vel_w)/self.Rw
        self.wheel_vel[3] = (vel_x*math.sin(math.pi/4 - math.pi/2) + vel_y*math.cos(math.pi/4 - math.pi/2) + self.L*vel_w)/self.Rw

        array_forPublish = Float64MultiArray(data=self.wheel_vel)    
        #rclpy.logging._root_logger.info(f"wheel vel : {self.wheel_vel}")
        self.publisher_.publish(array_forPublish)

def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
