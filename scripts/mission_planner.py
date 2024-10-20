#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

class MissionPlanner:
    def __init__(self):
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        
        self.current_pose = None
        self.current_path = None
        
        rospy.loginfo("Mission Planner initialized")

    def pose_callback(self, msg):
        self.current_pose = msg

    def update(self):
        if self.current_pose is None:
            return
        
        # Calculate desired path
        desired_path = Path()
        desired_path.header = self.current_pose.header
        desired_path.poses.append(self.current_pose)
        
        # Calculate error
        error = Twist()
        error.linear.x = desired_path.poses[0].pose.position.x - self.current_pose.pose.position.x
        error.linear.y = desired_path.poses[0].pose.position.y - self.current_pose.pose.position.y
        error.linear.z = desired_path.poses[0].pose.position.z - self.current_pose.pose.position.z
        
        # Calculate control output
        control_output = Twist()
        control_output.linear.x = error.linear.x
        control_output.linear.y = error.linear.y
        control_output.linear.z = error.linear.z
        
        # Publish control output
        self.cmd_vel_pub.publish(control_output)

if __name__ == '__main__':
    try:
        planner = MissionPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
