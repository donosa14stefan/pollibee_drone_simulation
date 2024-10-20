#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

class TrajectoryPlanner:
    def __init__(self):
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)
        self.pose_sub = rospy.Subscriber('pose', Pose Stamped, self.pose_callback)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        self.current_pose = None
        self.current_cmd_vel = None
        
        rospy.loginfo("Trajectory Planner initialized")

    def pose_callback(self, msg):
        self.current_pose = msg

    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg

    def update(self):
        if self.current_pose is None or self.current_cmd_vel is None:
            return
        
        # Calculate desired trajectory
        desired_trajectory = Path()
        desired_trajectory.header = self.current_pose.header
        desired_trajectory.poses.append(self.current_pose)
        
        # Calculate error
        error = Twist()
        error.linear.x = self.current_cmd_vel.linear.x - self.current_pose.pose.position.x
        error.linear.y = self.current_cmd_vel.linear.y - self.current_pose.pose.position.y
        error.linear.z = self.current_cmd_vel.linear.z - self.current_pose.pose.position.z
        
        # Calculate control output
        control_output = Twist()
        control_output.linear.x = error.linear.x
        control_output.linear.y = error.linear.y
        control_output.linear.z = error.linear.z
        
        # Publish control output
        self.path_pub.publish(desired_trajectory)

if __name__ == '__main__':
    try:
        planner = TrajectoryPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
