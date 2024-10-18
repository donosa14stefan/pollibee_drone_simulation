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
        
        # Parameters
        self.lower_color = np.array(rospy.get_param('~lower_color', [20, 100, 100]))  # Lower bound of flower color in HSV
        self.upper_color = np.array(rospy.get_param('~upper_color', [30, 255, 255]))  # Upper bound of flower color in HSV
        self.min_contour_area = rospy.get_param('~min_contour_area', 100)  # Minimum contour area to be considered a flower
        
        # Publishers and Subscribers
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.image_callback)
        self.flower_pub = rospy.Publisher("flower_positions", PoseArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("flower_detection_debug", Image, queue_size=10)
        
        rospy.loginfo("Flower Detector Node Initialized")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        flower_positions = self.detect_flowers(cv_image)
        self.publish_flower_positions(flower_positions, data.header)

    def detect_flowers(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create a mask for flower color
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        flower_positions = []
        
        for contour in contours:
            if cv2.contourArea(contour) > self.min_contour_area:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    # Append the position (you might need to transform these coordinates)
                    flower_positions.append((cX, cY))
                    
                    # Draw the contour and centroid on the image (for visualization)
                    cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
        
        # Publish debug image
        self.publish_debug_image(image)
        
        return flower_positions

    def publish_flower_positions(self, flower_positions, header):
        pose_array = PoseArray()
        pose_array.header = header
        
        for pos in flower_positions:
            pose = Pose()
            pose.position.x = pos[0]  # You might need to transform these coordinates
            pose.position.y = pos[1]  # to the appropriate frame of reference
            pose.position.z = 0       # Assuming flowers are at ground level
            pose_array.poses.append(pose)
        
        self.flower_pub.publish(pose_array)

        def publish_debug_image(self, image):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = FlowerDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
