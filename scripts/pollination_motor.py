#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class PollinationMotor:
    def __init__(self):
        rospy.init_node('pollination_motor')
        
        # Parameters
        self.pollination_duration = rospy.get_param('~pollination_duration', 2.0)
        
        # State variables
        self.is_pollinating = False
        
        # Publishers
        self.motor_state_pub = rospy.Publisher('pollination_motor_state', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Pollination Motor initialized")

    def cmd_vel_callback(self, msg):
        # Check if the drone has stopped (assuming it's near a plant)
        if abs(msg.linear.x) < 0.01 and abs(msg.linear.y) < 0.01 and abs(msg.linear.z) < 0.01:
            if not self.is_pollinating:
                self.start_pollination()
        else:
            self.stop_pollination()

    def start_pollination(self):
        self.is_pollinating = True
        self.motor_state_pub.publish(Bool(True))
        rospy.loginfo("Starting pollination")
        rospy.Timer(rospy.Duration(self.pollination_duration), self.stop_pollination, oneshot=True)

    def stop_pollination(self, event=None):
        if self.is_pollinating:
            self.is_pollinating = False
            self.motor_state_pub.publish(Bool(False))
            rospy.loginfo("Stopping pollination")

    def run(self):
        while not rospy.is_shutdown():
            # Publish current motor state
            self.motor_state_pub.publish(Bool(self.is_pollinating))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        motor = PollinationMotor()
        motor.run()
    except rospy.ROSInterruptException:
        pass
