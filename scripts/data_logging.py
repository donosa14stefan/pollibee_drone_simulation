#!/usr/bin/env python3

import rospy
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from datetime import datetime
import os

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger')
        
        # Parameters
        self.log_directory = rospy.get_param('~log_directory', 'logs')
        self.visualization_directory = rospy.get_param('~visualization_directory', 'visualizations')
        
        # Create directories if they don't exist
        os.makedirs(self.log_directory, exist_ok=True)
        os.makedirs(self.visualization_directory, exist_ok=True)
        
        self.pose_sub = rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        self.pollination_sub = rospy.Subscriber('pollinate', Bool, self.pollination_callback)
        
        # Use a timestamp in the filename to avoid overwriting
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = os.path.join(self.log_directory, f'pollination_log_{timestamp}.csv')
        self.log_file = open(log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'Pollination'])
        
        self.poses = []
        self.pollinations = []
        
        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo(f"Data Logger initialized. Logging to {log_filename}")
        
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
            rospy.loginfo(f"Pollination event logged at {timestamp}")
            
    def shutdown_hook(self):
        rospy.loginfo("Shutting down Data Logger. Closing log file and generating visualization.")
        self.log_file.close()
        self.visualize_data()
        
    def visualize_data(self):
        if not self.poses:
            rospy.logwarn("No pose data to visualize.")
            return
        
        timestamps, x, y, z = zip(*self.poses)
        
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot drone trajectory
        ax.plot(x, y, z, label='Drone Trajectory')
        
        # Plot pollination events
        if self.pollinations:
            pollination_x = [x[timestamps.index(min(timestamps, key=lambda t: abs(t-p)))] for p in self.pollinations]
            pollination_y = [y[timestamps.index(min(timestamps, key=lambda t: abs(t-p)))] for p in self.pollinations]
            pollination_z = [z[timestamps.index(min(timestamps, key=lambda t: abs(t-p)))] for p in self.pollinations]
            ax.scatter(pollination_x, pollination_y, pollination_z, c='r', marker='*', s=100, label='Pollination Events')
        
        ax.set_xlabel('X')
                ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Drone Trajectory and Pollination Events')
        ax.legend()
        
        # Add timestamp to visualization filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        visualization_filename = os.path.join(self.visualization_directory, f'pollination_visualization_{timestamp}.png')
        plt.savefig(visualization_filename)
        rospy.loginfo(f"Visualization saved to {visualization_filename}")
        
        # Display plot if running in an environment with a display
        if 'DISPLAY' in os.environ:
            plt.show()
        else:
            rospy.loginfo("No display detected. Skipping plot display.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = DataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
