#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.drone_vel = Twist()

    def joy_callback(self, joy_msg):
        self.drone_vel.linear.x = joy_msg.axes[1] * 0.5
        self.drone_vel.linear.y = joy_msg.axes[0] * 0.5
        self.drone_vel.linear.z = joy_msg.axes[3] * 0.5
        self.drone_vel.angular.z = joy_msg.axes[2] * 0.5
        self.cmd_vel_pub.publish(self.drone_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
