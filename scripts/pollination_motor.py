#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class PollinatorMotor:
    def __init__(self):
        self.node_name = "pollinator_motor"
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Publishers
        self.motor_cmd_pub = rospy.Publisher("motor_cmd", Float64, queue_size=10)

        # Subscribers
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

        # Initialize motor command
        self.motor_cmd = Float64()

        # Motor parameters
        self.max_motor_speed = 100.0  # Maximum motor speed
        self.speed_factor = 0.5  # Factor to convert linear velocity to motor speed

    def twist_callback(self, twist_msg):
        # Map twist message to motor command
        linear_speed = (abs(twist_msg.linear.x) + abs(twist_msg.linear.y) + abs(twist_msg.linear.z)) / 3
        motor_speed = linear_speed * self.speed_factor * self.max_motor_speed

        # Ensure motor speed is within limits
        self.motor_cmd.data = max(0, min(motor_speed, self.max_motor_speed))

        # Publish motor command
        self.motor_cmd_pub.publish(self.motor_cmd)

    def run(self):
        rospy.loginfo("Pollinator motor controller started")
        while not rospy.is_shutdown():
            # You can add any continuous control logic here if needed
            self.rate.sleep()

if __name__ == "__main__":
    try:
        motor = PollinatorMotor()
        motor.run()
    except rospy.ROSInterruptException:
        pass
