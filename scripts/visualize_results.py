#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Visualizer:
    def __init__(self):
        self.node_name = "visualizer"
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscribers
        self.image_sub = rospy.Subscriber("image_with_detections", Image, self.image_callback)

        # Create CvBridge object
        self.bridge = CvBridge()

    def image_callback(self, image_msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Display image with detections
        plt.imshow(cv_image)
        plt.axis("off")
        plt.show(block=False)
        plt.pause(0.01)

    def run(self):
        rospy.init_node(self.node_name)
        rospy.loginfo("Visualizer started")

        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    visualizer = Visualizer()
    visualizer.run()
