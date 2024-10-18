#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager')
        
        # Publishers
        self.mission_status_pub = rospy.Publisher('mission_status', String, queue_size=10)
        self.mission_command_pub = rospy.Publisher('mission_command', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('battery_level', Float32, self.battery_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('pollination_status', Bool, self.pollination_callback)
        rospy.Subscriber('base_station_command', String, self.base_station_callback)
        
        # State variables
        self.current_pose = None
        self.battery_level = None
        self.current_path = None
        self.is_pollinating = False
        self.mission_active = False
        
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.loginfo("Mission Manager initialized")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.check_mission_progress()

    def battery_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level < 20.0 and self.mission_active:
            self.abort_mission("LOW_BATTERY")

    def path_callback(self, msg):
        self.current_path = msg
        self.check_mission_progress()

    def pollination_callback(self, msg):
        self.is_pollinating = msg.data

    def base_station_callback(self, msg):
        command = msg.data
        if command == "START_MISSION":
            self.start_mission()
        elif command == "ABORT_MISSION":
            self.abort_mission("BASE_STATION_COMMAND")

    def start_mission(self):
        if not self.mission_active and self.battery_level > 90.0:
            self.mission_active = True
            self.mission_command_pub.publish("START")
            self.mission_status_pub.publish("Mission started")
        else:
            rospy.logwarn("Cannot start mission: Battery level too low or mission already active")

    def abort_mission(self, reason):
        self.mission_active = False
        self.mission_command_pub.publish("ABORT")
        self.mission_status_pub.publish(f"Mission aborted: {reason}")

    def check_mission_progress(self):
        if self.mission_active and self.current_path and self.current_pose:
            if len(self.current_path.poses) == 0:
                self.mission_status_pub.publish("Mission completed")
                self.mission_active = False
            else:
                progress = (1 - len(self.current_path.poses) / len(self.current_path.poses)) * 100
                self.mission_status_pub.publish(f"Mission progress: {progress:.2f}%")

    def run(self):
        while not rospy.is_shutdown():
            if self.mission_active:
                self.check_mission_progress()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        manager = MissionManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
