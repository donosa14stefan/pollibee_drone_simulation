#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class TrajectoryFollower:
    def __init__(self):
        rospy.init_node('trajectory_follower')
        
        # Parameters
        self.update_rate = rospy.get_param('~update_rate', 10)  # Hz
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)  # rad/s
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)  # m
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 0.5)  # m
        
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
        self.current_pose = msg.pose

       def get_lookahead_point(self):
        if not self.current_path or not self.current_pose:
            return None

        drone_position = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        for i in range(1, len(self.current_path.poses)):
            point = self.current_path.poses[i].pose.position
            point_position = np.array([point.x, point.y])
            
            if np.linalg.norm(point_position - drone_position) > self.lookahead_distance:
                return point
        
        return self.current_path.poses[-1].pose.position

    def calculate_control(self, lookahead_point):
        current_position = np.array([self.current_pose.position.x, self.current_pose.position.y])
        lookahead_position = np.array([lookahead_point.x, lookahead_point.y])
        
        distance = np.linalg.norm(lookahead_position - current_position)
        
        # Calculate desired heading
        desired_heading = np.arctan2(lookahead_position[1] - current_position[1],
                                     lookahead_position[0] - current_position[0])
        
        # Get current heading
        _, _, yaw = euler_from_quaternion([self.current_pose.orientation.x,
                                           self.current_pose.orientation.y,
                                           self.current_pose.orientation.z,
                                           self.current_pose.orientation.w])
        
        # Calculate heading error
        heading_error = desired_heading - yaw
        if heading_error > np.pi:
            heading_error -= 2 * np.pi
        elif heading_error < -np.pi:
            heading_error += 2 * np.pi
        
        # Calculate linear and angular velocities
        linear_velocity = min(self.max_linear_speed, distance)
        angular_velocity = min(self.max_angular_speed, abs(heading_error)) * np.sign(heading_error)
        
        return linear_velocity, angular_velocity

    def run(self):
        while not rospy.is_shutdown():
            if self.current_path and self.current_pose:
                lookahead_point = self.get_lookahead_point()
                if lookahead_point:
                    linear_vel, angular_vel = self.calculate_control(lookahead_point)
                    
                    cmd_vel = Twist()
                    cmd_vel.linear.x = linear_vel
                    cmd_vel.angular.z = angular_vel
                    
                    self.cmd_vel_pub.publish(cmd_vel)
                    
                    if np.linalg.norm([self.current_pose.position.x - self.current_path.poses[-1].pose.position.x,
                                       self.current_pose.position.y - self.current_path.poses[-1].pose.position.y]) < self.goal_tolerance:
                        rospy.loginfo("Reached final goal")
                        self.current_path = None
                else:
                    rospy.logwarn("No valid lookahead point found")
            else:
                # Stop the drone if there's no path to follow
                self.cmd_vel_pub.publish(Twist())
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = TrajectoryFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
