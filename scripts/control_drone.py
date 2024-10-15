#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

class DroneController:
    def __init__(self):
        self.node_name = "drone_controller"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Subscribers
        self.path_sub = rospy.Subscriber("planned_path", Path, self.path_callback)
        self.pose_sub = rospy.Subscriber("drone_pose", PoseStamped, self.pose_callback)
        self.yolo_sub = rospy.Subscriber("yolo_detections", Float32MultiArray, self.yolo_callback)

        # Initialize drone state
        self.drone_pose = PoseStamped()
        self.current_path = None
        self.current_goal = None
        self.yolo_detections = []

        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.goal_tolerance = 0.1

    def path_callback(self, path_msg):
        self.current_path = path_msg.poses
        if self.current_goal is None and self.current_path:
            self.current_goal = self.current_path.pop(0)

    def pose_callback(self, pose_msg):
        self.drone_pose = pose_msg

    def yolo_callback(self, yolo_msg):
        self.yolo_detections = yolo_msg.data

    def get_drone_yaw(self):
        orientation_q = self.drone_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def move_to_goal(self):
        if self.current_goal is None:
            return Twist()  # No movement if no goal

        # Calculate distance and angle to goal
        dx = self.current_goal.pose.position.x - self.drone_pose.pose.position.x
        dy = self.current_goal.pose.position.y - self.drone_pose.pose.position.y
        distance = np.sqrt(dx**2 + dy**2)

        desired_yaw = np.arctan2(dy, dx)
        current_yaw = self.get_drone_yaw()
        yaw_error = desired_yaw - current_yaw

        # Normalize yaw error to [-pi, pi]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

        # Create Twist message for movement
        twist = Twist()

        if distance > self.goal_tolerance:
            # Move towards goal
            twist.linear.x = min(self.linear_speed, 0.5 * distance)
            twist.angular.z = self.angular_speed * yaw_error
        else:
            # Goal reached, get next goal if available
            if self.current_path:
                self.current_goal = self.current_path.pop(0)
            else:
                self.current_goal = None

        return twist

    def avoid_obstacles(self, twist):
        # Simple obstacle avoidance based on YOLO detections
        if self.yolo_detections:
            # Assume yolo_detections is a list of [x, y, width, height] for each detection
            for detection in self.yolo_detections:
                x, y, width, height = detection
                # If obstacle is in the center of the image and close (large width/height)
                if 0.4 < x < 0.6 and (width > 0.2 or height > 0.2):
                    # Stop forward motion
                    twist.linear.x = 0
                    # Turn to avoid obstacle
                    twist.angular.z = self.angular_speed
                    break  # Only avoid one obstacle at a time

        return twist

    def run(self):
        rospy.loginfo("Drone controller started")
        while not rospy.is_shutdown():
            if self.current_goal:
                # Get movement command to reach goal
                twist = self.move_to_goal()
                # Modify movement to avoid obstacles
                twist = self.avoid_obstacles(twist)
                # Publish movement command
                self.cmd_vel_pub.publish(twist)
            else:
                # If no goal, hover in place
                self.cmd_vel_pub.publish(Twist())

            self.rate.sleep()

if __name__ == "__main__":
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
