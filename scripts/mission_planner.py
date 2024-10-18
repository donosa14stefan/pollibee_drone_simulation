#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import DBSCAN

class MissionPlanner:
    def __init__(self):
        rospy.init_node('mission_planner')
        
        # Parameters
        self.field_size = rospy.get_param('~field_size', [100, 100])  # meters
        self.altitude = rospy.get_param('~altitude', 10)  # meters
        self.waypoint_spacing = rospy.get_param('~waypoint_spacing', 10)  # meters
        self.clustering_epsilon = rospy.get_param('~clustering_epsilon', 5)  # meters
        self.min_samples = rospy.get_param('~min_samples', 3)
        
        # Publishers
        self.path_pub = rospy.Publisher('planned_path', Path, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('flower_detections', Float32MultiArray, self.flower_callback)
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        
        self.flower_positions = []
        self.current_pose = None
        
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.loginfo("Mission Planner initialized")

    def flower_callback(self, msg):
        self.flower_positions = np.array(msg.data).reshape(-1, 3)
        self.plan_mission()

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def generate_survey_pattern(self):
        x = np.arange(0, self.field_size[0], self.waypoint_spacing)
        y = np.arange(0, self.field_size[1], self.waypoint_spacing)
        xx, yy = np.meshgrid(x, y)
        survey_points = np.column_stack((xx.ravel(), yy.ravel()))
        return survey_points

    def cluster_flowers(self):
        if len(self.flower_positions) < self.min_samples:
            return self.flower_positions

        clustering = DBSCAN(eps=self.clustering_epsilon, min_samples=self.min_samples).fit(self.flower_positions[:, :2])
        cluster_centers = []
        for label in set(clustering.labels_):
            if label != -1:  # -1 is noise
                cluster_points = self.flower_positions[clustering.labels_ == label]
                cluster_centers.append(np.mean(cluster_points, axis=0))
        return np.array(cluster_centers)

    def optimize_path(self, points):
        if self.current_pose is None:
            start_point = np.array([0, 0])
        else:
            start_point = np.array([self.current_pose.position.x, self.current_pose.position.y])

        optimized_path = [start_point]
        remaining_points = points.copy()

                while len(remaining_points) > 0:
            distances = np.linalg.norm(remaining_points - optimized_path[-1], axis=1)
            nearest_index = np.argmin(distances)
            optimized_path.append(remaining_points[nearest_index])
            remaining_points = np.delete(remaining_points, nearest_index, axis=0)

        return np.array(optimized_path)

    def plan_mission(self):
        survey_points = self.generate_survey_pattern()
        clustered_flowers = self.cluster_flowers()
        
        all_points = np.vstack((survey_points, clustered_flowers[:, :2]))
        optimized_path = self.optimize_path(all_points)
        
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for point in optimized_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = Point(point[0], point[1], self.altitude)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        rospy.loginfo(f"Published mission path with {len(path_msg.poses)} waypoints")

    def run(self):
        while not rospy.is_shutdown():
            if self.current_pose is not None and len(self.flower_positions) > 0:
                self.plan_mission()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        planner = MissionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
