#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        
        # Parameters
        self.update_rate = rospy.get_param('~update_rate', 1)  # Hz
        self.plot_window = rospy.get_param('~plot_window', 100)  # Number of points to display
        
        # State variables
        self.drone_positions = []
        self.plant_positions = []
        self.pollination_events = []
        self.current_path = None
        
        # Set up the plot
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Drone Simulation Visualization')
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('plant_positions', PoseStamped, self.plant_callback)
        rospy.Subscriber('pollinate', Bool, self.pollination_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        
        self.rate = rospy.Rate(self.update_rate)
        rospy.loginfo("Visualizer Node Initialized")

    def drone_pose_callback(self, msg):
        self.drone_positions.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if len(self.drone_positions) > self.plot_window:
            self.drone_positions.pop(0)

    def plant_callback(self, msg):
        self.plant_positions.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def pollination_callback(self, msg):
        if msg.data:
            self.pollination_events.append(self.drone_positions[-1])

    def path_callback(self, msg):
        self.current_path = [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in msg.poses]

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
        if self.plant_positions:
            plants = np.array(self.plant_positions)
            self.ax.scatter(plants[:, 0], plants[:, 1], plants[:, 2], c='g', marker='^', label='Plants')

        # Plot pollination events
        if self.pollination_events:
            events = np.array(self.pollination_events)
            self.ax.scatter(events[:, 0], events[:, 1], events[:, 2], c='y', marker='*', s=200, label='Pollination Events')

        # Plot planned path
        if self.current_path:
            path = np.array(self.current_path)
            self.ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r--', label='Planned Path')

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
