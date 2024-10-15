#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        
        # Parameters
        self.min_distance = rospy.get_param('~min_distance', 0.5)  # Minimum distance to obstacle
        self.max_speed = rospy.get_param('~max_speed', 1.0)  # Maximum speed of the drone
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.laser_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.twist = Twist()
        
        rospy.loginfo("Obstacle Avoidance Node Initialized")

    def laser_callback(self, msg):
        # Get the minimum distance from the laser scan
        min_distance = min(msg.ranges)
        
        if min_distance < self.min_distance:
            # Obstacle detected, calculate avoidance vector
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            
            # Find the angle of the minimum distance reading
            min_angle = angle_min + msg.ranges.index(min_distance) * angle_increment
            
            # Calculate avoidance vector
            avoid_x = np.cos(min_angle + np.pi)
            avoid_y = np.sin(min_angle + np.pi)
            
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
