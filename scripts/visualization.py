#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        self.pose_sub = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)
        self.fig, self.ax = plt.subplots()

    def pose_callback(self, msg):
        self.ax.clear()
        self.ax.plot(msg.pose.position.x, msg.pose.position.y, 'bo')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        plt.draw()
        plt.pause(0.01)

    def path_callback(self, msg):
        self.ax.clear()
        for pose in msg.poses:
            self.ax.plot(pose.pose.position.x, pose.pose.position.y, 'ro')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        plt.draw()
        plt.pause(0.01)

    def run(self):
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    visualizer = Visualizer()
    visualizer.run()
