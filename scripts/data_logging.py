#!/usr/bin/env python3

import rospy
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class DataLogger:
    def __init__(self):
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.pollination_sub = rospy.Subscriber('pollination', Bool, self.pollination_callback)
        
        self.poses = []
        self.pollinations = []
        
        rospy.loginfo("Data Logger initialized")

    def pose_callback(self, msg):
        self.poses.append(msg)

    def pollination_callback(self, msg):
        if msg.data:
            self.pollinations.append(msg)

    def log_data(self):
        # Log data to CSV file
        with open('data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Pose", "Pollination"])
            for pose, pollination in zip(self.poses, self.pollinations):
                writer.writerow([pose, pollination])

    def plot_data(self):
        # Plot data
        plt.plot(self.poses)
        plt.plot(self.pollinations)
        plt.show()

if __name__ == '__main__':
    try:
        logger = DataLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
