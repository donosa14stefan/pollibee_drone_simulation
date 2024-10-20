#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

class DroneController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        self.wind_sub = rospy.Subscriber('wind', Twist, self.wind_callback)
        
        self.current_pose = None
        self.current_imu = None
        self.wind_vector = Twist()
        
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.Kp = rospy.get_param('~Kp', 1.0 )
        self.Kd = rospy.get_param('~Kd', 0.1)
        
        rospy.loginfo("Drone Controller initialized")

    def pose_callback(self, msg):
        self.current_pose = msg

    def imu_callback(self, msg):
        self.current_imu = msg

    def wind_callback(self, msg):
        self.wind_vector = msg

    def update(self):
        if self.current_pose is None or self.current_imu is None:
            return
        
        # Calculate desired velocity
        desired_velocity = Twist()
        desired_velocity.linear.x = self.Kp * (self.current_pose.pose.position.x - 0.5)
        desired_velocity.linear.y = self.Kp * (self.current_pose.pose.position.y - 0.5)
        desired_velocity.linear.z = self.Kp * (self.current_pose.pose.position.z - 1.0)
        
        # Calculate error
        error = Twist()
        error.linear.x = desired_velocity.linear.x - self.current_imu.linear_acceleration.x
        error.linear.y = desired_velocity.linear.y - self.current_imu.linear_acceleration.y
        error.linear.z = desired_velocity.linear.z - self.current_imu.linear_acceleration.z
        
        # Calculate control output
        control_output = Twist()
        control_output.linear.x = self.Kp * error.linear.x + self.Kd * (error.linear.x - self.wind_vector.linear.x)
        control_output.linear.y = self.Kp * error.linear.y + self.Kd * (error.linear.y - self.wind_vector.linear.y)
        control_output.linear.z = self.Kp * error.linear.z + self.Kd * (error.linear.z - self.wind_vector.linear.z)
        
        # Publish control output
        self.cmd_vel_pub.publish(control_output)

if __name__ == '__main__':
    try:
        controller = DroneController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
