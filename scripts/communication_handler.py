#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CommunicationHandler:
    def __init__(self):
        self.status_pub = rospy.Publisher('status', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)
        
        self.status_sub = rospy.Subscriber('status', String, self.status_callback)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', PoseStamped, self.cmd_vel_callback)
        self.path_sub = rospy.Subscriber('path', Path, self.path_callback)
        
        rospy.loginfo("Communication Handler initialized")

    def status_callback(self, msg):
        rospy.loginfo("Received status: %s", msg.data)

    def cmd_vel_callback(self, msg):
        rospy.loginfo("Received cmd_vel: %s", msg)

    def path_callback(self, msg):
        rospy.loginfo("Received path: %s", msg)

    def process_messages(self):
        # Process incoming messages
        pass

if __name__ == '__main__':
    try:
        handler = CommunicationHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
