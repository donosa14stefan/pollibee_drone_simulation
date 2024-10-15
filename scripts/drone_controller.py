#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Parameters
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)  # m/s
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.1)  # meters
        
        # Subscribers
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('pollinate_command', Bool, self.pollinate_callback)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.current_pose_pub = rospy.Publisher('drone_pose', PoseStamped, queue_size=10)
        
        # State variables
        self.current_pose = None
        self.target_pose = None
        self.is_pollinating = False
        
        rospy.loginfo("Drone Controller initialized")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.current_pose_pub.publish(msg)

    def path_callback(self, msg):
        if len(msg.poses) > 0:
            self.target_pose = msg.poses[-1].pose

    def pollinate_callback(self, msg):
        self.is_pollinating = msg.data
        if self.is_pollinating:
            rospy.loginfo("Starting pollination")
            self.execute_pollination()
        else:
            rospy.loginfo("Pollination complete")

    def execute_pollination(self):
        # Simulate pollination process
        rospy.sleep(2)  # Wait for 2 seconds
        self.is_pollinating = False

    def calculate_velocity(self):
        if self.current_pose is None or self.target_pose is None:
            return Twist()

        # Calculate position error
        position_error = np.array([
            self.target_pose.position.x - self.current_pose.position.x,
            self.target_pose.position.y - self.current_pose.position.y,
            self.target_pose.position.z - self.current_pose.position.z
        ])

        # Calculate orientation error
        current_orientation = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])
        target_orientation = euler_from_quaternion([
            self.target_pose.orientation.x,
            self.target_pose.orientation.y,
            self.target_pose.orientation.z,
            self.target_pose.orientation.w
        ])
        orientation_error = np.array(target_orientation) - np.array(current_orientation)

        # Calculate linear and angular velocities
        linear_velocity = np.clip(position_error, -self.max_velocity, self.max_velocity)
        angular_velocity = np.clip(orientation_error, -self.max_velocity, self.max_velocity)

        # Create Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity[0]
        cmd_vel.linear.y = linear_velocity[1]
            cmd_vel.linear.z = linear_velocity[2]
        cmd_vel.angular.x = angular_velocity[0]
        cmd_vel.angular.y = angular_velocity[1]
        cmd_vel.angular.z = angular_velocity[2]

        return cmd_vel

    def has_reached_target(self):
        if self.current_pose is None or self.target_pose is None:
            return False

        position_error = np.linalg.norm([
            self.target_pose.position.x - self.current_pose.position.x,
            self.target_pose.position.y - self.current_pose.position.y,
            self.target_pose.position.z - self.current_pose.position.z
        ])

        return position_error < self.position_tolerance

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            if not self.is_pollinating:
                if self.has_reached_target():
                    rospy.loginfo("Reached target position")
                    self.target_pose = None
                elif self.target_pose is not None:
                    cmd_vel = self.calculate_velocity()
                    self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
