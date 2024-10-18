#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        
        # Parameters
        self.min_distance = rospy.get_param('~min_distance', 0.5)  # Minimum distance to obstacle
        self.max_speed = rospy.get_param('~max_speed', 1.0)  # Maximum speed of the drone
        self.avoid_distance = rospy.get_param('~avoid_distance', 1.0)  # Distance to keep from obstacles
        self.field_of_view = rospy.get_param('~field_of_view', 180)  # Laser scanner field of view in degrees
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.avoidance_pose_pub = rospy.Publisher('avoidance_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.laser_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.twist = Twist()
        
        rospy.loginfo("Obstacle Avoidance Node Initialized")

    def laser_callback(self, msg):
        # Get the minimum distance from the laser scan
        ranges = np.array(msg.ranges)
        min_distance = np.min(ranges)
        
        if min_distance < self.min_distance:
            # Obstacle detected, calculate avoidance vector
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            
            # Find the angle of the minimum distance reading
            min_angle_index = np.argmin(ranges)
            min_angle = angle_min + min_angle_index * angle_increment
            
            # Calculate avoidance vector
            avoid_x = np.cos(min_angle + np.pi)
            avoid_y = np.sin(min_angle + np.pi)
            
            # Calculate desired position to avoid obstacle
            desired_x = avoid_x * self.avoid_distance
            desired_y = avoid_y * self.avoid_distance
            
            # Create and publish avoidance pose
            avoidance_pose = PoseStamped()
            avoidance_pose.header.stamp = rospy.Time.now()
            avoidance_pose.header.frame_id = "base_link"
            avoidance_pose.pose.position.x = desired_x
            avoidance_pose.pose.position.y = desired_y
            avoidance_pose.pose.position.z = 0
                        self.avoidance_pose_pub.publish(avoidance_pose)
            
            # Set linear velocity inversely proportional to distance
            self.twist.linear.x = self.max_speed * (min_distance / self.min_distance)
            
            # Set angular velocity to turn away from obstacle
            self.twist.angular.z = -avoid_y  # Negative because ROS uses left-hand rule
        else:
            # No obstacle, continue forward
            self.twist.linear.x = self.max_speed
            self.twist.angular.z = 0
        
        # Publish velocity command
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
