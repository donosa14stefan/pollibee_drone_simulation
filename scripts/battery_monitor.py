#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class BatteryMonitor:
    def __init__(self):
        self.battery_pub = rospy.Publisher('battery', Float32, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        self.current_battery = 100.0
        self.discharge_rate = 0.1
        
        rospy.loginfo("Battery Monitor initialized")

    def cmd_vel_callback(self, msg):
        self.current_battery -= self.discharge_rate * msg.linear.x def update(self):
        self.battery_pub.publish(self.current_battery)

if __name__ == '__main__':
    try:
        monitor = BatteryMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
