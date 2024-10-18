#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Path

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Parameters
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        self.control_mode = rospy.get_param('~control_mode', 'manual')  # 'manual' or 'autonomous'
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.drone_pose_pub = rospy.Publisher('drone_pose', PoseStamped, queue_size=10)
        
        # Subscribers
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.path_sub = rospy.Subscriber('planned_path', Path, self.path_callback)
        
        # State variables
        self.drone_vel = Twist()
        self.current_pose = PoseStamped()
        self.planned_path = None
        self.current_waypoint_index = 0
        
        rospy.loginfo("Drone Controller initialized")
    
    def joy_callback(self, joy_msg):
        if self.control_mode == 'manual':
            self.drone_vel.linear.x = joy_msg.axes[1] * self.max_linear_speed
            self.drone_vel.linear.y = joy_msg.axes[0] * self.max_linear_speed
            self.drone_vel.linear.z = joy_msg.axes[3] * self.max_linear_speed
            self.drone_vel.angular.z = joy_msg.axes[2] * self.max_angular_speed
            
            # Publish velocity command
            self.cmd_vel_pub.publish(self.drone_vel)
        
        # Check for mode switch button
        if joy_msg.buttons[7] == 1:  # Assuming button 7 is the mode switch
            self.toggle_control_mode()
    
    def path_callback(self, path_msg):
        self.planned_path = path_msg
        self.current_waypoint_index = 0
        rospy.loginfo("Received new planned path")
    
    def toggle_control_mode(self):
        if self.control_mode == 'manual':
            self.control_mode = 'autonomous'
            rospy.loginfo("Switched to autonomous control mode")
        else:
            self.control_mode = 'manual'
            rospy.loginfo("Switched to manual control mode")
    
    def update_pose(self):
        # In a real implementation, this would come from odometry or a localization system
        # For simulation purposes, we'll just update it based on the current velocity
        self.current_pose.header.stamp = rospy.Time.now()
        self.current_pose.pose.position.x += self.drone_vel.linear.x * 0.1  # Assuming 10Hz update rate
        self.current_pose.pose.position.y += self.drone_vel.linear.y * 0.1
        self.current_pose.pose.position.z += self.drone_vel.linear.z * 0.1
        
        # Publish updated pose
        self.drone_pose_pub.publish(self.current_pose)
    
       def autonomous_control(self):
        if self.planned_path is None or self.current_waypoint_index >= len(self.planned_path.poses):
            self.drone_vel = Twist()  # Stop if no path or reached end of path
            return
        
        target_pose = self.planned_path.poses[self.current_waypoint_index].pose
        
        # Calculate direction to target
        dx = target_pose.position.x - self.current_pose.pose.position.x
        dy = target_pose.position.y - self.current_pose.pose.position.y
        dz = target_pose.position.z - self.current_pose.pose.position.z
        
        # Simple proportional control
        self.drone_vel.linear.x = dx * 0.5
        self.drone_vel.linear.y = dy * 0.5
        self.drone_vel.linear.z = dz * 0.5
        
        # Limit velocity
        self.drone_vel.linear.x = max(min(self.drone_vel.linear.x, self.max_linear_speed), -self.max_linear_speed)
        self.drone_vel.linear.y = max(min(self.drone_vel.linear.y, self.max_linear_speed), -self.max_linear_speed)
        self.drone_vel.linear.z = max(min(self.drone_vel.linear.z, self.max_linear_speed), -self.max_linear_speed)
        
        # Check if we've reached the current waypoint
        if (dx**2 + dy**2 + dz**2)**0.5 < 0.1:  # Within 10cm
            self.current_waypoint_index += 1
            rospy.loginfo(f"Reached waypoint {self.current_waypoint_index}")
        
        # Publish velocity command
        self.cmd_vel_pub.publish(self.drone_vel)
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.control_mode == 'autonomous':
                self.autonomous_control()
            self.update_pose()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
