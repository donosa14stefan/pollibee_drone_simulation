#!/usr/bin/env python3

import rospy
import csv
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class DataLogger:
    def __init__(self):
        self.node_name = "data_logger"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscribers
        self.image_sub = rospy.Subscriber("yolo_detections", Image, self.image_callback)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

        # CSV file setup
        self.csv_file = open("data_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(["Timestamp", "Image_Seq", "Linear_X", "Linear_Y", "Linear_Z", "Angular_Z"])

        # Data storage
        self.latest_image_seq = None
        self.latest_twist = None

    def image_callback(self, image_msg):
        self.latest_image_seq = image_msg.header.seq
        self.log_data()

    def twist_callback(self, twist_msg):
        self.latest_twist = twist_msg
        self.log_data()

    def log_data(self):
        if self.latest_image_seq is not None and self.latest_twist is not None:
            timestamp = rospy.Time.now().to_sec()
            self.csv_writer.writerow([
                timestamp,
                self.latest_image_seq,
                self.latest_twist.linear.x,
                self.latest_twist.linear.y,
                self.latest_twist.linear.z,
                self.latest_twist.angular.z
            ])
            self.csv_file.flush()  # Ensure data is written to file immediately

    def run(self):
        rospy.loginfo("Data logger started")
        while not rospy.is_shutdown():
            self.rate.sleep()

    def __del__(self):
        self.csv_file.close()

if __name__ == "__main__":
    try:
        logger = DataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
