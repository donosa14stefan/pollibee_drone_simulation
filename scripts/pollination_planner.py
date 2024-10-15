#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray

class PollinationPlanner:
    def __init__(self):
        rospy.init_node('pollination_planner')
        
        # Parameters
        self.field_size = rospy.get_param('~field_size', [10, 10, 2])  # x, y, z in meters
        self.grid_size = rospy.get_param('~grid_size', 1.0)  # meters
        self.hover_height = rospy.get_param('~hover_height', 1.5)  # meters
        
        # Create grid
        self.create_grid()
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('yolo_detections', Float32MultiArray, self.yolo_detection_callback)
        
        # Publishers
        self.path_pub = rospy.Publisher('planned_path', Path, queue_size=10)
        self.markers_pub = rospy.Publisher('visualization_markers', MarkerArray, queue_size=10)
        self.pollinate_pub = rospy.Publisher('pollinate_command', Bool, queue_size=10)
        
        # State variables
        self.drone_pose = None
        self.current_target = None
        self.plants_to_pollinate = []
        self.pollinated_plants = set()
        
        rospy.loginfo("Pollination Planner initialized")

    def create_grid(self):
        x = np.arange(0, self.field_size[0], self.grid_size)
        y = np.arange(0, self.field_size[1], self.grid_size)
        self.grid = np.stack(np.meshgrid(x, y)).T.reshape(-1, 2)
        np.random.shuffle(self.grid)  # Randomize grid order

    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose
        if self.current_target is None:
            self.plan_next_move()

    def yolo_detection_callback(self, msg):
        detections = np.array(msg.data).reshape(-1, 6)  # [x, y, w, h, confidence, class_id]
        for detection in detections:
            if detection[5] == 0:  # Assuming 0 is the class ID for plants
                plant_position = Point(detection[0], detection[1], self.hover_height)
                if plant_position not in self.pollinated_plants:
                    self.plants_to_pollinate.append(plant_position)

    def plan_next_move(self):
        if self.plants_to_pollinate:
            self.current_target = self.plants_to_pollinate.pop(0)
        elif self.grid.size > 0:
            next_point = self.grid[0]
            self.grid = self.grid[1:]
            self.current_target = Point(next_point[0], next_point[1], self.hover_height)
        else:
            rospy.loginfo("Pollination complete")
            return

        path = self.create_path(self.drone_pose.position, self.current_target)
        self.path_pub.publish(path)
        self.publish_markers()

    def create_path(self, start, end):
        path = Path()
        path.header.frame_id = "world"
        path.header.stamp = rospy.Time.now()

        # Add start point
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position = start
        path.poses.append(pose)

        # Add end point
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position = end
        path.poses.append(pose)

        return path

    def publish_markers(self):
        marker_array = MarkerArray()

        # Add marker for current target
        if self.current_target:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "current_target"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = self.current_target
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # Add markers for plants to pollinate
        for i, plant in enumerate(self.plants_to_pollinate):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "plants_to_pollinate"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = plant
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)

    def check_reached_target(self):
        if self.drone_pose and self.current_target:
            distance = np.linalg.norm([
                self.drone_pose.position.x - self.current_target.x,
                self.drone_pose.position.y - self.current_target.y,
                self.drone_pose.position.z - self.current_target.z
            ])
            if distance < 0.1:  # Threshold distance in meters
                return True
        return False

    def pollinate(self):
        pollinate_cmd = Bool()
        pollinate_cmd.data = True
        self.pollinate_pub.publish(pollinate_cmd)
        rospy.sleep(2)  # Wait for pollination to complete
        self.pollinated_plants.add(self.current_target)
        self.current_target = None

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.check_reached_target():
                self.pollinate()
                self.plan_next_move()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = PollinationPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
