#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPlanner:
    def __init__(self):
        self.node_name = "path_planner"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(1)  # 1 Hz

        # Publishers
        self.path_pub = rospy.Publisher("planned_path", Path, queue_size=10)

        # Path parameters
        self.path_length = 20
        self.field_size = 10  # Assume a 10x10 field

    def generate_path(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for i in range(self.path_length):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = np.random.uniform(0, self.field_size)
            pose.pose.position.y = np.random.uniform(0, self.field_size)
            pose.pose.position.z = 1.0  # Fixed height
            path.poses.append(pose)

        return path

    def run(self):
        rospy.loginfo("Path planner started")
        while not rospy.is_shutdown():
            path = self.generate_path()
            self.path_pub.publish(path)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
