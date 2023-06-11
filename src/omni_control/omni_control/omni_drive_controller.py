import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

wheel_velocities = [0,0,0,0]

class OmniDriveController(Node):
    def __init__(self):
        super().__init__('omni_drive_controller')

        # Wheel controller publishers
        self.wheel1_pub = self.create_publisher(Float64, '/robot/Revolute_34_controller/command', 1)
        self.wheel2_pub = self.create_publisher(Float64, '/robot/Revolute_42_controller/command', 1)
        self.wheel3_pub = self.create_publisher(Float64, '/robot/Revolute_43_controller/command', 1)
        self.wheel4_pub = self.create_publisher(Float64, '/robot/Revolute_44_controller/command', 1)

        # Subscribe to the cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        r = 0.21  # Wheel radius in meters
        R = 0.2   # Distance from the center of the robot to the wheel in meters


        wheel_velocities[0] = (vx*math.sin(math.pi/4            ) + vy*math.cos(math.pi/4            ) + R*omega)/r
        wheel_velocities[1] = (vx*math.sin(math.pi/4 + math.pi/2) + vy*math.cos(math.pi/4 + math.pi/2) + R*omega)/r
        wheel_velocities[2] = (vx*math.sin(math.pi/4 - math.pi)   + vy*math.cos(math.pi/4 - math.pi)   + R*omega)/r
        wheel_velocities[3] = (vx*math.sin(math.pi/4 - math.pi/2) + vy*math.cos(math.pi/4 - math.pi/2) + R*omega)/r
        # Publish the calculated wheel velocities
        self.wheel1_pub.publish(Float64(data=wheel_velocities[0]))
        self.wheel2_pub.publish(Float64(data=wheel_velocities[1]))
        self.wheel3_pub.publish(Float64(data=wheel_velocities[2]))
        self.wheel4_pub.publish(Float64(data=wheel_velocities[3]))

def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

