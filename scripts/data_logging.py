#!/usr/bin/env python

import rospy
import numpy as np
import csv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DataLogger:
    def __init__(self):
        self.node_name = "data_logger"
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscribers
        self.image_sub = rospy.Subscriber("image_with_detections", Image, self.image_callback)

        # Create CvBridge object
        self.bridge = CvBridge()

        # Create CSV writer
        self.csv_file = open("data_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

    def image_callback(self, image_msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Extract features from image (e.g. detection bounding boxes, class labels)
        features = self.extract_features(cv_image)

        # Write features to CSV file
        self.csv_writer.writerow(features)

    def extract_features(self, cv_image):
        # TO DO: implement feature extraction logic here
        # For example, extract detection bounding boxes and class labels
        features = []
        # ...
        return features

    def run(self):
        rospy.init_node(self.node_name)
        rospy.loginfo("Data logger started")

        while not rospy.is_shutdown():
            self.rate.sleep()

    def __del__(self):
        self.csv_file.close()

if __name__ == "__main__":
    logger = DataLogger()
    logger.run()
