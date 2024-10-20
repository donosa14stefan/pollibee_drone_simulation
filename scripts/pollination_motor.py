#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class PollinationMotor:
    def __init__(self):
        self.motor_pub = rospy.Publisher('pollination_motor', Bool, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        self.is_pollinating = False
        
        rospy.loginfo("Pollination Motor initialized")

    def cmd_vel_callback(self, msg):
        if msg.linear.x > 0.5:
            self.start_pollination()
        else:
            self.stop_pollination()

    def start_pollination(self):
        if not self.is_pollinating:
            self.is_pollinating = True
            self.motor_pub.publish(Bool(True))
            rospy.loginfo("Starting pollination")

    def stop_pollination(self):
        if self.is_pollinating:
            self.is_pollinating = False
            self.motor_pub.publish(Bool(False))
            rospy.loginfo("Stopping pollination")

if __name__ == '__main__':
    try:
        motor = PollinationMotor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
