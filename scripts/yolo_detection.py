#!/usr/bin/env python

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class YOLODetector:
    def __init__(self):
        self.node_name = "yolo_detector"
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publishers
        self.image_pub = rospy.Publisher("image_with_detections", Image, queue_size=10)

        # Subscribers
        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)

        # Load YOLO model
        self.net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
        self.classes = []
        with open("coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # Create CvBridge object
        self.bridge = CvBridge()

    def image_callback(self, image_msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Get image dimensions
        (H, W) = cv_image.shape[:2]

        # Create blob from image
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

        # Set input blob for the network
        self.net.setInput(blob)

        # Run object detection
        outs = self.net.forward(self.output_layers)

        # Initialize lists to store detected objects
        class_ids = []
        confidences = []
        boxes = []

        # Loop through each detection
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * W)
                    center_y = int(detection[1] * H)
                    w = int(detection[2] * W)
                    h = int(detection[3] * H)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-max suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Draw bounding boxes and labels
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = "{}: {:.2f}%".format(self.classes[class_ids[i]], confidences[i] * 100)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x, y + 30), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)

        # Publish image with detections
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(image_msg)

    def run(self):
        rospy.init_node(self.node_name)
        rospy.loginfo("YOLO detector started")

        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    detector = YOLODetector()
    detector.run()
