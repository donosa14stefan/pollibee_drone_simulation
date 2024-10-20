#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose

class FlowerDetector:
    def __init__(self):
        rospy.init_node('flower_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.flower_pub = rospy.Publisher('/detected_flowers', PoseArray, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        detected_flowers = self.detect_flowers(cv_image)
        self.publish_flowers(detected_flowers)

    def detect_flowers(self, image):
        # Implementează aici logica de detectare a florilor
        # Aceasta poate include segmentare pe bază de culoare, detecție de contururi, etc.
        # Returnează o listă de poziții (x, y) ale florilor detectate
        # Această implementare este un placeholder
        return [(100, 100), (200, 200)]  # Exemplu de returnat

    def publish_flowers(self, flower_positions):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "camera_frame"

        for pos in flower_positions:
            pose = Pose()
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = 0  # Asumăm că toate florile sunt la aceeași înălțime
            pose_array.poses.append(pose)

        self.flower_pub.publish(pose_array)

if __name__ == '__main__':
    detector = FlowerDetector()
    rospy.spin()
