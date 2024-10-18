#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')
        
        # Parameters
        self.field_size = rospy.get_param('~field_size', [10, 10])  # Field size in meters
        self.grid_resolution = rospy.get_param('~grid_resolution', 0.5)  # Grid resolution in meters
        self.flight_altitude = rospy.get_param('~flight_altitude', 2.0)  # Flight altitude in meters
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon', 0.5)  # DBSCAN clustering epsilon
        self.min_samples = rospy.get_param('~min_samples', 3)  # DBSCAN minimum samples
        
        # Publishers
        self.path_pub = rospy.Publisher('planned_path', Path, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('flower_positions', Float32MultiArray, self.flower_callback)
        rospy.Subscriber('drone_pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('avoidance_pose', PoseStamped, self.avoidance_callback)
        
        self.flower_positions = []
        self.drone_pose = None
        self.avoidance_pose = None
        self.grid = None
        
        self.create_grid()
        
        rospy.loginfo("Path Planner Node Initialized")

    def create_grid(self):
        x = np.arange(0, self.field_size[0], self.grid_resolution)
        y = np.arange(0, self.field_size[1], self.grid_resolution)
        self.grid = np.array(np.meshgrid(x, y)).T.reshape(-1, 2)

    def flower_callback(self, msg):
        self.flower_positions = np.array(msg.data).reshape(-1, 3)
        self.plan_path()

    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose
        if not self.flower_positions:
            self.plan_path()

    def avoidance_callback(self, msg):
        self.avoidance_pose = msg.pose
        self.plan_path()

    def cluster_flowers(self):
        if len(self.flower_positions) < 2:
            return self.flower_positions

        clustering = DBSCAN(eps=self.cluster_epsilon, min_samples=self.min_samples).fit(self.flower_positions[:, :2])
        cluster_centers = []
        for label in set(clustering.labels_):
            if label != -1:  # -1 is noise
                cluster_points = self.flower_positions[clustering.labels_ == label]
                cluster_centers.append(np.mean(cluster_points, axis=0))
        return np.array(cluster_centers)

    def optimize_path(self, points):
        n = len(points)
        cost_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                cost_matrix[i, j] = np.linalg.norm(points[i] - points[j])
        
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return points[col_ind]

    def plan_path(self):
        if self.drone_pose is None:
            return

        path = Path()
        path.header.stamp = rospy.Time.now()
                path.header.frame_id = "map"

        # Start from current drone position
        current_pos = Point(self.drone_pose.position.x, self.drone_pose.position.y, self.flight_altitude)
        path.poses.append(self.create_pose_stamped(current_pos))

        # If there's an avoidance pose, add it to the path
        if self.avoidance_pose:
            avoidance_point = Point(self.avoidance_pose.position.x, self.avoidance_pose.position.y, self.flight_altitude)
            path.poses.append(self.create_pose_stamped(avoidance_point))

        # Cluster flower positions
        clustered_flowers = self.cluster_flowers()

        # Optimize path through clustered flowers
        if len(clustered_flowers) > 0:
            optimized_path = self.optimize_path(clustered_flowers)
            for flower in optimized_path:
                flower_point = Point(flower[0], flower[1], self.flight_altitude)
                path.poses.append(self.create_pose_stamped(flower_point))

        # Add grid points to cover the entire field
        for grid_point in self.grid:
            grid_pose = Point(grid_point[0], grid_point[1], self.flight_altitude)
            path.poses.append(self.create_pose_stamped(grid_pose))

        # Optimize the entire path
        optimized_poses = self.optimize_path(np.array([pose.pose.position for pose in path.poses]))
        optimized_path = Path()
        optimized_path.header = path.header
        for pose in optimized_poses:
            optimized_path.poses.append(self.create_pose_stamped(Point(*pose)))

        # Publish the optimized path
        self.path_pub.publish(optimized_path)
        rospy.loginfo(f"Published path with {len(optimized_path.poses)} waypoints")

    def create_pose_stamped(self, point):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position = point
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.plan_path()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
