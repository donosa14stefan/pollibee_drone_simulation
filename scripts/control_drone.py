#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class DroneController:
    def __init__(self):
        self.node_name = "drone_controller"
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

        # Initialize drone velocity
        self.drone_vel = Twist()

    def joy_callback(self, joy_msg):
        # Map joystick axes to drone velocity
        self.drone_vel.linear.x = joy_msg.axes[1] * 0.5
        self.drone_vel.linear.y = joy_msg.axes[0] * 0.5
        self.drone_vel.linear.z = joy_msg.axes[3] * 0.5
        self.drone_vel.angular.z = joy_msg.axes[2] * 0.5

        # Publish drone velocity
        self.cmd_vel_pub.publish(self.drone_vel)

    def run(self):
        rospy.init_node(self.node_name)
        rospy.loginfo("Drone controller started")

        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    controller = DroneController()
    controller.run()
