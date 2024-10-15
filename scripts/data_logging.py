#!/usr/bin/env python3

import rospy
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger')
        
        self.pose_sub = rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        self.pollination_sub = rospy.Subscriber('pollinate', Bool, self.pollination_callback)
        
        self.log_file = open('pollination_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'Pollination'])
        
        self.poses = []
        self.pollinations = []
        
        rospy.on_shutdown(self.shutdown_hook)
        
    def pose_callback(self, msg):
        pose = msg.pose.position
        timestamp = msg.header.stamp.to_sec()
        self.csv_writer.writerow([timestamp, pose.x, pose.y, pose.z, ''])
        self.poses.append((timestamp, pose.x, pose.y, pose.z))
        
    def pollination_callback(self, msg):
        if msg.data:
            timestamp = rospy.Time.now().to_sec()
            self.csv_writer.writerow([timestamp, '', '', '', 'Pollination'])
            self.pollinations.append(timestamp)
            
    def shutdown_hook(self):
        self.log_file.close()
        self.visualize_data()
        
    def visualize_data(self):
        timestamps, x, y, z = zip(*self.poses)
        
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot drone trajectory
        ax.plot(x, y, z, label='Drone Trajectory')
        
        # Plot pollination events
        pollination_x = [x[timestamps.index(p)] for p in self.pollinations]
        pollination_y = [y[timestamps.index(p)] for p in self.pollinations]
        pollination_z = [z[timestamps.index(p)] for p in self.pollinations]
        ax.scatter(pollination_x, pollination_y, pollination_z, c='r', marker='*', s=100, label='Pollination Events')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Drone Trajectory and Pollination Events')
        ax.legend()
        
        plt.savefig('pollination_visualization.png')
        plt.show()

if __name__ == '__main__':
    try:
        logger = DataLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
