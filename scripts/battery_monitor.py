#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

class BatteryMonitor:
    def __init__(self):
        rospy.init_node('battery_monitor')
        
        # Publishers
        self.battery_level_pub = rospy.Publisher('battery_level', Float32, queue_size=10)
        self.low_battery_pub = rospy.Publisher('low_battery_alert', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # Parameters
        self.initial_battery = rospy.get_param('~initial_battery', 100.0)  # %
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)  # %
        self.battery_drain_rate = rospy.get_param('~battery_drain_rate', 0.01)  # % per second
        self.movement_drain_factor = rospy.get_param('~movement_drain_factor', 2.0)  # Drain multiplier when moving
        
        self.battery_level = self.initial_battery
        self.is_moving = False
        
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.loginfo("Battery Monitor initialized")

    def cmd_vel_callback(self, msg):
        # Check if drone is moving
        self.is_moving = abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.linear.z) > 0.01 or abs(msg.angular.z) > 0.01

    def update_battery(self):
        drain = self.battery_drain_rate
        if self.is_moving:
            drain *= self.movement_drain_factor
        
        self.battery_level -= drain
        self.battery_level = max(0, min(100, self.battery_level))  # Ensure battery level is between 0 and 100
        
        # Publish battery level
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_level_pub.publish(battery_msg)
        
        # Check for low battery
        if self.battery_level < self.low_battery_threshold:
            low_battery_msg = Bool()
            low_battery_msg.data = True
            self.low_battery_pub.publish(low_battery_msg)
            rospy.logwarn(f"Low battery alert! Current level: {self.battery_level:.2f}%")

    def run(self):
        while not rospy.is_shutdown():
            self.update_battery()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        monitor = BatteryMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
