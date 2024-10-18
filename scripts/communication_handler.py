#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CommunicationHandler:
    def __init__(self):
        rospy.init_node('communication_handler')
        
        # Publishers
        self.status_pub = rospy.Publisher('drone_status', String, queue_size=10)
        self.command_pub = rospy.Publisher('base_station_command', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('battery_level', Float32, self.battery_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('pollination_status', Bool, self.pollination_callback)
        rospy.Subscriber('base_station_input', String, self.base_station_callback)
        
        # State variables
        self.current_pose = None
        self.battery_level = None
        self.current_path = None
        self.is_pollinating = False
        
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.loginfo("Communication Handler initialized")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.publish_status()

    def battery_callback(self, msg):
        self.battery_level = msg.data
        self.publish_status()

    def path_callback(self, msg):
        self.current_path = msg
        self.publish_status()

    def pollination_callback(self, msg):
        self.is_pollinating = msg.data
        self.publish_status()

    def base_station_callback(self, msg):
        command = msg.data
        if command == "START_MISSION":
            self.command_pub.publish("START")
        elif command == "ABORT_MISSION":
            self.command_pub.publish("ABORT")
        elif command == "RETURN_TO_BASE":
            self.command_pub.publish("RTB")
        else:
            rospy.logwarn(f"Unknown command received: {command}")

    def publish_status(self):
        status_msg = String()
        status = "Drone Status:\n"
        
        if self.current_pose:
            status += f"Position: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, {self.current_pose.position.z:.2f})\n"
        
        if self.battery_level is not None:
            status += f"Battery: {self.battery_level:.2f}%\n"
        
        if self.current_path:
            status += f"Path progress: {len(self.current_path.poses)} waypoints remaining\n"
        
        status += f"Pollinating: {'Yes' if self.is_pollinating else 'No'}\n"
        
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_status()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        handler = CommunicationHandler()
        handler.run()
    except rospy.ROSInterruptException:
        pass
