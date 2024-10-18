#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager')
        
        # Publishers
        self.start_mission_pub = rospy.Publisher('start_mission', Bool, queue_size=10)
        self.abort_mission_pub = rospy.Publisher('abort_mission', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('battery_level', Float32, self.battery_callback)
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('pollination_status', Bool, self.pollination_callback)
        
        # Mission parameters
        self.battery_threshold = rospy.get_param('~battery_threshold', 20.0)  # %
        self.mission_complete = False
        self.current_pose = None
        self.planned_path = None
        self.is_pollinating = False
        
        rospy.loginfo("Mission Manager initialized")

    def battery_callback(self, msg):
        if msg.data < self.battery_threshold:
            rospy.logwarn("Low battery! Aborting mission.")
            self.abort_mission()

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def path_callback(self, msg):
        self.planned_path = msg

    def pollination_callback(self, msg):
        self.is_pollinating = msg.data

    def start_mission(self):
        start_msg = Bool()
        start_msg.data = True
        self.start_mission_pub.publish(start_msg)
        rospy.loginfo("Mission started")

    def abort_mission(self):
        abort_msg = Bool()
        abort_msg.data = True
        self.abort_mission_pub.publish(abort_msg)
        rospy.loginfo("Mission aborted")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        self.start_mission()
        
        while not rospy.is_shutdown() and not self.mission_complete:
            if self.planned_path and self.current_pose:
                if self.is_path_complete():
                    self.mission_complete = True
                    rospy.loginfo("Mission completed successfully!")
                
            rate.sleep()

    def is_path_complete(self):
        if not self.planned_path or not self.current_pose:
            return False
        
        last_point = self.planned_path.poses[-1].pose.position
        current_position = self.current_pose.position
        
        distance = ((last_point.x - current_position.x) ** 2 +
                    (last_point.y - current_position.y) ** 2 +
                    (last_point.z - current_position.z) ** 2) ** 0.5
        
        return distance < 0.1  # Consider path complete if within 10 cm of last point

if __name__ == '__main__':
    try:
        manager = MissionManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
