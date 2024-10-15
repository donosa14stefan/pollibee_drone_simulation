#!/usr/bin/env python3

import rospy
import csv
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger')
        
        # Parameters
        self.log_file = rospy.get_param('~log_file', 'simulation_log.csv')
        
        # State variables
        self.current_pose = None
        self.pollination_count = 0
        self.image_count = 0
        
        # CSV setup
        self.csv_file = open(self.log_file, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'Pollination_Count', 'Image_Count'])
        
        # Subscribers
        rospy.Subscriber('drone_pose', Pose, self.pose_callback)
        rospy.Subscriber('pollination_motor_state', Bool, self.pollination_callback)
        rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        
        self.rate = rospy.Rate(1)  # 1 Hz logging
        rospy.loginfo("Data Logger initialized")

    def pose_callback(self, msg):
        self.current_pose = msg

    def pollination_callback(self, msg):
        if msg.data:
            self.pollination_count += 1

    def image_callback(self, msg):
        self.image_count += 1

    def log_data(self):
        if self.current_pose is not None:
            timestamp = rospy.get_time()
            self.csv_writer.writerow([
                timestamp,
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z,
                self.pollination_count,
                self.image_count
            ])

    def run(self):
        while not rospy.is_shutdown():
            self.log_data()
            self.rate.sleep()

    def __del__(self):
        self.csv_file.close()
        rospy.loginfo("Data logging completed. File saved: {}".format(self.log_file))

if __name__ == '__main__':
    try:
        logger = DataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
