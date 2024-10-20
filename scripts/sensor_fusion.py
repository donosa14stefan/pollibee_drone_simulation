#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped

class SensorFusion:
    def __init__(self):
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        
        self.current_pose = None
        self.current_imu = None
        self.current_scan = None
        
        rospy.loginfo("Sensor Fusion initialized")

    def imu_callback(self, msg):
        self.current_imu = msg

    def scan_callback(self, msg):
        self.current_scan = msg

    def update(self):
        if self.current_imu is None or self.current_scan is None:
            return
        
        # Fuse sensor data
        fused_pose = PoseStamped()
        fused_pose.header = self.current_imu.header
        fused_pose.pose.position.x = self.current_imu.linear_acceleration.x
        fused_pose.pose.position.y = self.current_imu.linear_acceleration.y
        fused_pose.pose.position.z = self.current_imu.linear_acceleration.z
        
        # Publish fused pose
        self.pose_pub.publish(fused_pose)

if __name__ == '__main__':
    try:
        fusion = SensorFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
