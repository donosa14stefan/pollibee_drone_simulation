#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Path

class TrajectoryPlanner:
    def __init__(self):
        rospy.init_node('trajectory_planner')
        self.flower_sub = rospy.Subscriber('flower_detections', PoseArray, self.flower_callback)
        self.path_pub = rospy.Publisher('drone_path', Path, queue_size=10)
        self.current_flowers = []

    def flower_callback(self, msg):
        self.current_flowers = msg.poses
        self.plan_trajectory()

    def plan_trajectory(self):
        # Implementați un algoritm simplu pentru a crea o cale prin toate florile
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        # Sortați florile după distanță față de origine (puteți îmbunătăți acest algoritm)
        sorted_flowers = sorted(self.current_flowers, key=lambda pose: np.linalg.norm([pose.position.x, pose.position.y, pose.position.z]))

        for flower in sorted_flowers:
            path.poses.append(flower)

        self.path_pub.publish(path)

if __name__ == '__main__':
    try:
        planner = TrajectoryPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
