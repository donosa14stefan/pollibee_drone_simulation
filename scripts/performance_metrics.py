#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

class PerformanceMetrics:
    def __init__(self):
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        
        self.current_pose = None
        self.previous_pose = None
        
        rospy.loginfo("Performance Metrics initialized")

    def pose_callback(self, msg):
        self.current_pose = msg

    def update(self):
        if self.current_pose is None or self.previous_pose is None:
            return
        
        # Calculate distance traveled
        distance_traveled = np.linalg.norm([
            self.current_pose.pose.position.x - self.previous_pose.pose.position.x,
            self.current_pose.pose.position.y - self.previous_pose.pose.position.y,
            self.current_pose.pose.position.z - self.previous_pose.pose.position.z
        ])
        
        # Calculate velocity
        velocity = distance_traveled / (self.current_pose.header.stamp - self.previous_pose.header.stamp).to_sec()
        
        # Calculate acceleration
        acceleration = (velocity - self.previous_pose.linear_acceleration.x) / (self.current_pose.header.stamp - self.previous_pose.header.stamp).to_sec()
        
        # Publish performance metrics
        rospy.loginfo(f"Distance Traveled: {distance_traveled:.2f} m")
        rospy.loginfo(f"Velocity: {velocity:.2f} m/s")
        rospy.loginfo(f"Acceleration: {acceleration:.2f} m/s^2")

if __name__ == '__main__':
    try:
        metrics = PerformanceMetrics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
