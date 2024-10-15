#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class YoloDetector:
    def __init__(self):
        self.node_name = "yolo_detector"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Load YOLO model
        self.net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
        self.classes = []
        with open("coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # ROS publishers and subscribers
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.detections_pub = rospy.Publisher("yolo_detections", Float32MultiArray, queue_size=10)
        self.image_pub = rospy.Publisher("yolo_image", Image, queue_size=10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        detections = self.detect_objects(cv_image)
        self.publish_detections(detections)
        self.publish_image(cv_image, detections)

    def detect_objects(self, img):
        height, width, _ = img.shape
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        detections = []
        for i in range(len(boxes)):
            if i in indexes:
                detections.append([boxes[i], confidences[i], class_ids[i]])

        return detections
    def publish_detections(self, detections):
        msg = Float32MultiArray()
        for det in detections:
            box, confidence, class_id = det
            x, y, w, h = box
            msg.data.extend([x, y, w, h, confidence, class_id])
        self.detections_pub.publish(msg)

    def publish_image(self, img, detections):
        for det in detections:
            box, confidence, class_id = det
            x, y, w, h = box
            label = f"{self.classes[class_id]}: {confidence:.2f}"
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def run(self):
        rospy.loginfo("YOLO detector started")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        detector = YoloDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
