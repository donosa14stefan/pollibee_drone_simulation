#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray, Pose

class FlowerDetector:
    def __init__(self):
        rospy.init_node('flower_detector')
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.image_callback)
        self.flower_pub = rospy.Publisher("flower_positions", PoseArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("flower_detection_image", Image, queue_size=10)
        
        # Parameters for flower detection
        self.lower_color = np.array(rospy.get_param('~lower_color', [20, 100, 100]))  # Lower bound of flower color in HSV
        self.upper_color = np.array(rospy.get_param('~upper_color', [30, 255, 255]))  # Upper bound of flower color in HSV
        self.min_contour_area = rospy.get_param('~min_contour_area', 100)  # Minimum contour area to be considered a flower
        
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
        
        # Apply morphological operations to remove noise
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        flower_poses = PoseArray()
        flower_poses.header = data.header
        
        debug_image = cv_image.copy()
        
        for contour in contours:
            if cv2.contourArea(contour) > self.min_contour_area:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    # Create a Pose message for the flower position
                    flower_pose = Pose()
                    flower_pose.position.x = cX
                    flower_pose.position.y = cY
                    flower_pose.position.z = 0  # Assuming flowers are at ground level
                    flower_poses.poses.append(flower_pose)
                    
                    # Draw the contour and centroid on the debug image
                    cv2.drawContours(debug_image, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(debug_image, (cX, cY), 7, (255, 255, 255), -1)
        
        # Publish the flower positions
        self.flower_pub.publish(flower_poses)
        
                # Publish the debug image
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = FlowerDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
