#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class DroneController:
    def __init__(self):
        self.node_name = "drone_controller"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

        # Initialize drone velocity
        self.drone_vel = Twist()

        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0

    def joy_callback(self, joy_msg):
        # Map joystick axes to drone velocity
        self.drone_vel.linear.x = joy_msg.axes[1] * self.linear_speed
        self.drone_vel.linear.y = joy_msg.axes[0] * self.linear_speed
        self.drone_vel.linear.z = joy_msg.axes[3] * self.linear_speed
        self.drone_vel.angular.z = joy_msg.axes[2] * self.angular_speed

        # Publish drone velocity
        self.cmd_vel_pub.publish(self.drone_vel)

    def run(self):
        rospy.loginfo("Drone controller started")
        while not rospy.is_shutdown():
            # You can add any continuous control logic here if needed
            self.rate.sleep()

if __name__ == "__main__":
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
