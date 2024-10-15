#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector')

        # Parameters
        self.yolo_config = rospy.get_param('~yolo_config', 'path/to/yolo/config.cfg')
        self.yolo_weights = rospy.get_param('~yolo_weights', 'path/to/yolo/weights.weights')
        self.yolo_classes = rospy.get_param('~yolo_classes', 'path/to/yolo/classes.txt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.4)

        # Load YOLO network
        self.net = cv2.dnn.readNetFromDarknet(self.yolo_config, self.yolo_weights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # Load classes
        with open(self.yolo_classes, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribers
        rospy.Subscriber('drone_camera/image_raw', Image, self.image_callback)

        # Publishers
        self.detections_pub = rospy.Publisher('plant_detections', Float32MultiArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher('yolo_debug_image', Image, queue_size=10)

        rospy.loginfo("YOLO Detector initialized")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get image dimensions
        height, width = cv_image.shape[:2]

        # Create blob from image
        blob = cv2.dnn.blobFromImage(cv_image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        # Get output layer names
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # Run forward pass
        outputs = self.net.forward(output_layers)

        # Process detections
        class_ids = []
        confidences = []
        boxes = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.confidence_threshold:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    class_ids.append(class_id)
                                        confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        # Apply non-maximum suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)

        # Prepare detections message
        detections_msg = Float32MultiArray()
        debug_image = cv_image.copy()

        for i in indices:
            i = i[0]
            box = boxes[i]
            x, y, w, h = box
            label = f"{self.classes[class_ids[i]]}: {confidences[i]:.2f}"
            
            # Add detection to message
            detections_msg.data.extend([float(x), float(y), float(w), float(h), float(class_ids[i]), float(confidences[i])])
            
            # Draw bounding box and label on debug image
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(debug_image, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections
        self.detections_pub.publish(detections_msg)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        self.debug_image_pub.publish(debug_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
