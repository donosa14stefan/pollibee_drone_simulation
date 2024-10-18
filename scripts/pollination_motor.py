#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class PollinationMotor:
    def __init__(self):
        rospy.init_node('pollination_motor')
        
        # Parameters
        self.pollination_duration = rospy.get_param('~pollination_duration', 2.0)
        self.motor_speed = rospy.get_param('~motor_speed', 100)  # RPM
        
        # Publishers
        self.motor_state_pub = rospy.Publisher('pollination_motor_state', Bool, queue_size=10)
        self.joint_state_pub = rospy.Publisher('pollination_joint_state', JointState, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('pollinate', Bool, self.pollinate_callback)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        self.is_pollinating = False
        self.last_cmd_vel = Twist()
        
        self.rate = rospy.Rate(50)  # 50 Hz
        rospy.loginfo("Pollination Motor Node Initialized")

    def pollinate_callback(self, msg):
        if msg.data and not self.is_pollinating:
            self.start_pollination()
        elif not msg.data and self.is_pollinating:
            self.stop_pollination()

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        if self.is_pollinating and (msg.linear.x != 0 or msg.linear.y != 0 or msg.linear.z != 0 or msg.angular.z != 0):
            self.stop_pollination()

    def start_pollination(self):
        self.is_pollinating = True
        self.motor_state_pub.publish(Bool(True))
        rospy.loginfo("Starting pollination motor")
        rospy.Timer(rospy.Duration(self.pollination_duration), self.stop_pollination, oneshot=True)

    def stop_pollination(self, event=None):
        if self.is_pollinating:
            self.is_pollinating = False
            self.motor_state_pub.publish(Bool(False))
            rospy.loginfo("Stopping pollination motor")

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['pollination_motor']
        joint_state.position = [0.0]  # Assume fixed position
        joint_state.velocity = [self.motor_speed if self.is_pollinating else 0.0]
        joint_state.effort = [0.0]  # Assume no effort measurement
        self.joint_state_pub.publish(joint_state)

        def run(self):
        while not rospy.is_shutdown():
            self.publish_joint_state()
            self.motor_state_pub.publish(Bool(self.is_pollinating))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        motor = PollinationMotor()
        motor.run()
    except rospy.ROSInterruptException:
        pass
