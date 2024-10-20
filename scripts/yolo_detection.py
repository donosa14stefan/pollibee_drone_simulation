#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class YOLODetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.image_callback)
        self.detection_pub = rospy.Publisher("detections", Float32MultiArray, queue_size=10)
        
        self.yolo = YOLO("yolov8n.pt")
        
        rospy.loginfo("YOLO Detector initialized")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # Detect objects
        results = self.yolo(cv_image)
        
        # Publish detections
        detection_msg = Float32MultiArray()
        detection_msg.data = results.xyxy[0].tolist()
        self.detection_pub.publish(detection_msg)

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
