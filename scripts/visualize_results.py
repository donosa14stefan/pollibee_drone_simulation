#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Visualizer:
    def __init__(self):
        self.node_name = "visualizer"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = rospy.Subscriber("yolo_detections", Image, self.image_callback)

        # OpenCV window
        cv2.namedWindow("Pollibee Drone Simulation", cv2.WINDOW_NORMAL)

    def image_callback(self, image_msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Display image
            cv2.imshow("Pollibee Drone Simulation", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def run(self):
        rospy.loginfo("Visualizer started")
        while not rospy.is_shutdown():
            self.rate.sleep()

    def __del__(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        visualizer = Visualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
