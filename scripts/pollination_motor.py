#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class PollinatorMotor:
    def __init__(self):
        self.node_name = "pollinator_motor"
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publishers
        self.motor_cmd_pub = rospy.Publisher("motor_cmd", Float64, queue_size=10)

        # Subscribers
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

        # Initialize motor command
        self.motor_cmd = Float64()

    def twist_callback(self, twist_msg):
        # Map twist message to motor command
        self.motor_cmd.data = twist_msg.linear.x * 0.5

        # Publish motor command
        self.motor_cmd_pub.publish(self.motor_cmd)

    def run(self):
        rospy.init_node(self.node_name)
        rospy.loginfo("Pollinator motor started")

        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    motor = PollinatorMotor()
    motor.run()
