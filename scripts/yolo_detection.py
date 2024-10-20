import torch
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose

class YOLOv5Detector:
    def __init__(self):
        rospy.init_node('yolov5_detector')
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.classes = [0]  # Asumăm că clasa 0 este pentru flori
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.detection_pub = rospy.Publisher('/flower_detections', PoseArray, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image)
            detections = results.pandas().xyxy[0]
            
            pose_array = PoseArray()
            pose_array.header = msg.header

            for _, detection in detections.iterrows():
                if detection['class'] == 0:  # Clasa pentru flori
                    pose = Pose()
                    pose.position.x = (detection['xmin'] + detection['xmax']) / 2
                    pose.position.y = (detection['ymin'] + detection['ymax']) / 2
                    pose.position.z = 0  # Asumăm că toate florile sunt la aceeași înălțime
                    pose_array.poses.append(pose)

            self.detection_pub.publish(pose_array)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = YOLOv5Detector()
    detector.run()
