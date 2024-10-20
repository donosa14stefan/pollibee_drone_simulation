#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

class SimulationInterface:
    def __init__(self):
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        
        self.current_pose = None
        
        rospy.loginfo("Simulation Interface initialized")

    def step(self):
        # Simulate step
        self.current_pose = PoseStamped()
        self.current_pose.header.stamp = rospy.Time.now()
        self.current_pose.pose.position.x = np.random.uniform(-1.0, 1.0)
        self.current_pose.pose.position.y = np.random.uniform(-1.0, 1.0)
        self.current_pose.pose.position.z = np.random.uniform(-1.0, 1.0)
        
        # Publish pose
        self.pose_pub.publish(self.current_pose)

if __name__ == '__main__':
    try:
        interface = SimulationInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
