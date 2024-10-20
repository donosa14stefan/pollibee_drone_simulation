#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        
        self.current_scan = None
        
        rospy.loginfo("Obstacle Avoidance initialized")

    def scan_callback(self, msg):
        self.current_scan = msg

    def check_obstacles(self):
        if self.current_scan is None:
            return
        
        # Check for obstacles
        obstacles = []
        for i in range(len(self.current_scan.ranges)):
            if self.current_scan.ranges[i] < 0.5:
                obstacles.append(i)
        
        # Calculate control output
        control_output = Twist()
        if obstacles:
            control_output.linear.x = -0.5
            control_output.angular.z = 0.5
        else:
            control_output.linear.x = 0.5
            control_output.angular.z = 0.0
        
        # Publish control output
        self.cmd_vel_pub.publish(control_output)

if __name__ == '__main__':
    try:
        avoidance = ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
