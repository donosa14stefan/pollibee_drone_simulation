#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Range

class PollinationController:
    def __init__(self):
        rospy.init_node('pollination_controller')
        
        # Parameters
        self.pollination_distance = rospy.get_param('~pollination_distance', 0.5)  # meters
        self.pollination_duration = rospy.get_param('~pollination_duration', 2.0)  # seconds
        self.hover_altitude = rospy.get_param('~hover_altitude', 1.5)  # meters
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pollinate_pub = rospy.Publisher('pollinate', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('flower_positions', PoseStamped, self.flower_callback)
        rospy.Subscriber('drone_pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('height', Range, self.height_callback)
        
        self.flower_position = None
        self.drone_position = None
        self.current_height = None
        self.is_pollinating = False
        
        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Pollination Controller Node Initialized")

    def flower_callback(self, msg):
        self.flower_position = msg.pose.position

    def drone_pose_callback(self, msg):
        self.drone_position = msg.pose.position
        self.check_pollination()

    def height_callback(self, msg):
        self.current_height = msg.range

    def check_pollination(self):
        if self.flower_position is None or self.drone_position is None or self.current_height is None:
            return
        
        distance = np.linalg.norm([
            self.flower_position.x - self.drone_position.x,
            self.flower_position.y - self.drone_position.y
        ])

        if distance <= self.pollination_distance and abs(self.current_height - self.hover_altitude) < 0.1:
            self.pollinate()

    def pollinate(self):
        if not self.is_pollinating:
            self.is_pollinating = True
            rospy.loginfo("Starting pollination")
            
            # Stop the drone
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            # Activate pollination mechanism
            pollinate_msg = Bool()
            pollinate_msg.data = True
            self.pollinate_pub.publish(pollinate_msg)
            
            # Wait for pollination duration
            rospy.sleep(self.pollination_duration)
            
            # Deactivate pollination mechanism
            pollinate_msg.data = False
            self.pollinate_pub.publish(pollinate_msg)
            
            self.is_pollinating = False
            rospy.loginfo("Pollination complete")

    def run(self):
        while not rospy.is_shutdown():
            if not self.is_pollinating:
                self.check_pollination()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PollinationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
