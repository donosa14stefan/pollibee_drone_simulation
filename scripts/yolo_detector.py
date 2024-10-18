#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray

class YOLOv11Detector:
    def __init__(self):
        rospy.init_node('yolo_detector')
        
        # Load parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.4)
        self.yolo_model = rospy.get_param('~yolo_model', 'yolov11.weights')
        self.yolo_config = rospy.get_param('~yolo_config', 'yolov11.cfg')
        self.classes_file = rospy.get_param('~classes_file', 'coco.names')

        # Load YOLOv11 network
        self.net = cv2.dnn.readNet(self.yolo_model, self.yolo_config)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        
        # Load classes
        with open(self.classes_file, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        
        # Get output layer names
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Publishers
        self.detection_pub = rospy.Publisher('plant_detections', PoseArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher('detection_image', Image, queue_size=10)

        # Subscribers
        rospy.Subscriber('camera/image_raw', Image, self.image_callback)

        rospy.loginfo("YOLOv11 Detector initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Could not convert image: %s", str(e))
            return

        # Perform detection
        detections = self.detect_plants(cv_image)

        # Publish detections
        self.publish_detections(detections, msg.header)

        # Draw detections on image and publish
        annotated_image = self.draw_detections(cv_image, detections)
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))

        def detect_plants(self, image):
        height, width, _ = image.shape
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
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
                if confidence > self.confidence_threshold:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)

        detections = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                class_id = class_ids[i]
                confidence = confidences[i]
                if self.classes[class_id] in ['plant', 'flower']:  # Detect plants and flowers
                    detections.append([x, y, w, h, confidence, class_id])

        return detections

    def publish_detections(self, detections, header):
        pose_array = PoseArray()
        pose_array.header = header

        for detection in detections:
            x, y, w, h, confidence, class_id = detection
            pose = Pose()
            pose.position.x = x + w/2  # center x
            pose.position.y = y + h/2  # center y
            pose.position.z = 0  # assuming plants are on the ground
            pose_array.poses.append(pose)

        self.detection_pub.publish(pose_array)

    def draw_detections(self, image, detections):
        for detection in detections:
            x, y, w, h, confidence, class_id = detection
            label = f'{self.classes[class_id]}: {confidence:.2f}'
            color = (0, 255, 0)  # Green for plants/flowers
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return image

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = YOLOv11Detector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
