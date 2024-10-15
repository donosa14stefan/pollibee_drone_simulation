#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

class TrajectoryFollower:
    def __init__(self):
        rospy.init_node('trajectory_follower')
        
        # Parameters
        self.update_rate = rospy.get_param('~update_rate', 10)  # Hz
        self.max_speed = rospy.get_param('~max_speed', 1.0)  # m/s
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)  # m
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        
        self.rate = rospy.Rate(self.update_rate)
        self.current_path = None
        self.current_pose = None
        self.current_goal = None
        
        rospy.loginfo("Trajectory Follower Node Initialized")

    def path_callback(self, msg):
        self.current_path = msg
        if self.current_path and self.current_path.poses:
            self.current_goal = self.current_path.poses[0]

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_distance_to_goal(self):
        if self.current_pose is None or self.current_goal is None:
            return float('inf')
        return np.linalg.norm(
            np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z]) -
            np.array([self.current_goal.pose.position.x, self.current_goal.pose.position.y, self.current_goal.pose.position.z])
        )

    def run(self):
        while not rospy.is_shutdown():
            if self.current_path and self.current_pose:
                distance_to_goal = self.get_distance_to_goal()
                
                if distance_to_goal < self.goal_tolerance:
                    # We've reached the current goal, move to the next one
                    self.current_path.poses.pop(0)
                    if self.current_path.poses:
                        self.current_goal = self.current_path.poses[0]
                    else:
                        self.current_goal = None
                        rospy.loginfo("Reached final goal")
                
                if self.current_goal:
                    # Calculate direction to goal
                    direction = np.array([
                        self.current_goal.pose.position.x - self.current_pose.pose.position.x,
                        self.current_goal.pose.position.y - self.current_pose.pose.position.y,
                        self.current_goal.pose.position.z - self.current_pose.pose.position.z
                                    ])
                    
                    # Normalize direction vector
                    direction = direction / np.linalg.norm(direction)
                    
                    # Create velocity command
                    cmd_vel = Twist()
                    cmd_vel.linear.x = direction[0] * self.max_speed
                    cmd_vel.linear.y = direction[1] * self.max_speed
                    cmd_vel.linear.z = direction[2] * self.max_speed
                    
                    # Publish velocity command
                    self.cmd_vel_pub.publish(cmd_vel)
                else:
                    # No goal, stop the drone
                    self.cmd_vel_pub.publish(Twist())
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = TrajectoryFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
