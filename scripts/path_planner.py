#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.field_size = rospy.get_param('~field_size', [10, 10])  # [width, height] in meters
        self.altitude = rospy.get_param('~altitude', 2.0)  # meters
        self.waypoint_spacing = rospy.get_param('~waypoint_spacing', 1.0)  # meters

    def generate_path(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        x_points = np.arange(0, self.field_size[0], self.waypoint_spacing)
        y_points = np.arange(0, self.field_size[1], self.waypoint_spacing)

        for i, x in enumerate(x_points):
            y_range = y_points if i % 2 == 0 else reversed(y_points)
            for y in y_range:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = self.altitude
                path.poses.append(pose)

        return path

    def run(self):
        rate = rospy.Rate(1)  # Hz
        while not rospy.is_shutdown():
            path = self.generate_path()
            self.path_pub.publish(path)
            rate.sleep()

if __name__ == '__main__':
    planner = PathPlanner()
    planner.run()
