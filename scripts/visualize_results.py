#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        
        # State variables
        self.drone_positions = []
        self.plant_positions = []
        
        # Set up the plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Drone Simulation Visualization')
        
        # Subscribers
        rospy.Subscriber('drone_pose', Pose, self.drone_pose_callback)
        rospy.Subscriber('plant_detections', Float32MultiArray, self.plant_detection_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Visualizer initialized")

    def drone_pose_callback(self, msg):
        self.drone_positions.append([msg.position.x, msg.position.y, msg.position.z])
        if len(self.drone_positions) > 100:  # Keep only last 100 positions
            self.drone_positions.pop(0)

    def plant_detection_callback(self, msg):
        self.plant_positions = np.array(msg.data).reshape(-1, 5)[:, :3]  # Keep only x, y, z

        def update_plot(self):
        self.ax.clear()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Drone Simulation Visualization')
        
        # Plot drone path
        if self.drone_positions:
            drone_path = np.array(self.drone_positions)
            self.ax.plot(drone_path[:, 0], drone_path[:, 1], drone_path[:, 2], 'b-', label='Drone Path')
            self.ax.scatter(drone_path[-1, 0], drone_path[-1, 1], drone_path[-1, 2], c='r', s=100, marker='o', label='Current Position')

        # Plot plant positions
        if len(self.plant_positions) > 0:
            self.ax.scatter(self.plant_positions[:, 0], self.plant_positions[:, 1], self.plant_positions[:, 2], c='g', marker='^', label='Plants')

        self.ax.legend()
        plt.draw()
        plt.pause(0.001)

    def run(self):
        while not rospy.is_shutdown():
            self.update_plot()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = Visualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
