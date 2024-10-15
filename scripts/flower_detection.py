#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

class FlowerDetector:
    def __init__(self):
        rospy.init_node('flower_detector')
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.image_callback)
        self.flower_pub = rospy.Publisher("flower_positions", PoseStamped, queue_size=10)
        
        # Parameters for flower detection
        self.lower_color = np.array([20, 100, 100])  # Lower bound of flower color in HSV
        self.upper_color = np.array([30, 255, 255])  # Upper bound of flower color in HSV
        
        rospy.loginfo("Flower Detector Node Initialized")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create a mask for flower color
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Calculate centroid of the contour            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Create a PoseStamped message for the flower position
                flower_pose = PoseStamped()
                flower_pose.header = data.header
                flower_pose.pose.position.x = cX  # You might need to transform these coordinates
                flower_pose.pose.position.y = cY  # to the appropriate frame of reference
                flower_pose.pose.position.z = 0   # Assuming flowers are at ground level
                
                # Publish the flower position
                self.flower_pub.publish(flower_pose)
                
                # Draw the contour and centroid on the image (for visualization)
                cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
        
        # Display the result (you might want to publish this as a ROS topic instead)
        cv2.imshow("Flower Detection", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = FlowerDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
