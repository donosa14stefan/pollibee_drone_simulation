#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Parameters
        self.max_speed = rospy.get_param('~max_speed', 1.0)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.1)
        self.yaw_tolerance = rospy.get_param('~yaw_tolerance', 0.1)
        self.hover_height = rospy.get_param('~hover_height', 1.5)
        
        # State variables
        self.current_pose = None
        self.target_pose = None
        self.is_pollinating = False
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.reached_target_pub = rospy.Publisher('reached_target', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('pollinate', Bool, self.pollinate_callback)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        rospy.loginfo("Drone Controller Initialized")

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def path_callback(self, msg):
        if msg.poses:
            self.target_pose = msg.poses[0].pose
        else:
            self.target_pose = None

    def pollinate_callback(self, msg):
        self.is_pollinating = msg.data

    def calculate_control(self):
        if self.current_pose is None or self.target_pose is None:
            return Twist()

        # Calculate position error
        error_x = self.target_pose.position.x - self.current_pose.position.x
        error_y = self.target_pose.position.y - self.current_pose.position.y
        error_z = self.target_pose.position.z - self.current_pose.position.z

        # Calculate yaw error
        target_yaw = np.arctan2(2.0 * (self.target_pose.orientation.w * self.target_pose.orientation.z + 
                                       self.target_pose.orientation.x * self.target_pose.orientation.y),
                                1.0 - 2.0 * (self.target_pose.orientation.y * self.target_pose.orientation.y + 
                                             self.target_pose.orientation.z * self.target_pose.orientation.z))
        current_yaw = np.arctan2(2.0 * (self.current_pose.orientation.w * self.current_pose.orientation.z + 
                                        self.current_pose.orientation.x * self.current_pose.orientation.y),
                                 1.0 - 2.0 * (self.current_pose.orientation.y * self.current_pose.orientation.y + 
                                              self.current_pose.orientation.z * self.current_pose.orientation.z))
        error_yaw = target_yaw - current_yaw

        # Create control command
        cmd_vel = Twist()
        cmd_vel.linear.x = np.clip(error_x, -self.max_speed, self.max_speed)
        cmd_vel.linear.y = np.clip(error_y, -self.max_speed, self.max_speed)
               cmd_vel.linear.z = np.clip(error_z, -self.max_speed, self.max_speed)
        cmd_vel.angular.z = np.clip(error_yaw, -self.max_speed, self.max_speed)

        return cmd_vel

    def has_reached_target(self):
        if self.current_pose is None or self.target_pose is None:
            return False

        position_error = np.linalg.norm([
            self.target_pose.position.x - self.current_pose.position.x,
            self.target_pose.position.y - self.current_pose.position.y,
            self.target_pose.position.z - self.current_pose.position.z
        ])

        yaw_error = abs(np.arctan2(2.0 * (self.target_pose.orientation.w * self.target_pose.orientation.z + 
                                          self.target_pose.orientation.x * self.target_pose.orientation.y),
                                   1.0 - 2.0 * (self.target_pose.orientation.y * self.target_pose.orientation.y + 
                                                self.target_pose.orientation.z * self.target_pose.orientation.z)) - 
                        np.arctan2(2.0 * (self.current_pose.orientation.w * self.current_pose.orientation.z + 
                                          self.current_pose.orientation.x * self.current_pose.orientation.y),
                                   1.0 - 2.0 * (self.current_pose.orientation.y * self.current_pose.orientation.y + 
                                                self.current_pose.orientation.z * self.current_pose.orientation.z)))

        return position_error < self.position_tolerance and yaw_error < self.yaw_tolerance

    def hover(self):
        if self.current_pose is None:
            return Twist()

        error_z = self.hover_height - self.current_pose.position.z
        cmd_vel = Twist()
        cmd_vel.linear.z = np.clip(error_z, -self.max_speed, self.max_speed)
        return cmd_vel

    def run(self):
        while not rospy.is_shutdown():
            if self.is_pollinating:
                cmd_vel = self.hover()
            elif self.target_pose is not None:
                if self.has_reached_target():
                    self.reached_target_pub.publish(Bool(True))
                    self.target_pose = None
                    cmd_vel = self.hover()
                else:
                    cmd_vel = self.calculate_control()
            else:
                cmd_vel = self.hover()

            self.cmd_vel_pub.publish(cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
