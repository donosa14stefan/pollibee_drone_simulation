#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')
        
        # Parameters
        self.update_rate = rospy.get_param('~update_rate', 1)  # Hz
        
        # Publishers and Subscribers
        self.path_pub = rospy.Publisher('planned_path', Path, queue_size=10)
        rospy.Subscriber('plant_positions', PoseStamped, self.plant_callback)
        
        self.rate = rospy.Rate(self.update_rate)
        self.plant_positions = []
        
        rospy.loginfo("Path Planning Node Initialized")

    def plant_callback(self, msg):
        self.plant_positions.append(msg.pose.position)

    def plan_path(self):
        if len(self.plant_positions) < 2:
            return None

        # Simple nearest neighbor algorithm
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        current_pos = self.plant_positions[0]
        unvisited = self.plant_positions[1:]

        while unvisited:
            pose = PoseStamped()
            pose.pose.position = current_pos
            path.poses.append(pose)

            distances = [np.linalg.norm(np.array([p.x, p.y, p.z]) - np.array([current_pos.x, current_pos.y, current_pos.z])) for p in unvisited]
            nearest_index = np.argmin(distances)
            current_pos = unvisited.pop(nearest_index)

        # Add the last position
        pose = PoseStamped()
        pose.pose.position = current_pos
        path.poses.append(pose)

        return path

    def run(self):
        while not rospy.is_shutdown():
            path = self.plan_path()
            if path:
                self.path_pub.publish(path)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
