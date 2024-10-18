#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix

class BatteryMonitor:
    def __init__(self):
        rospy.init_node('battery_monitor')
        
        # Parameters
        self.initial_battery_level = rospy.get_param('~initial_battery_level', 100.0)
        self.base_drain_rate = rospy.get_param('~base_drain_rate', 0.01)  # % per second
        self.altitude_drain_factor = rospy.get_param('~altitude_drain_factor', 0.001)  # % per meter
        self.speed_drain_factor = rospy.get_param('~speed_drain_factor', 0.02)  # % per m/s
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)
        self.critical_battery_threshold = rospy.get_param('~critical_battery_threshold', 10.0)
        
        # State variables
        self.battery_level = self.initial_battery_level
        self.current_altitude = 0.0
        self.current_speed = 0.0
        
        # Publishers
        self.battery_pub = rospy.Publisher('battery_level', Float32, queue_size=10)
        self.battery_status_pub = rospy.Publisher('battery_status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('velocity', TwistStamped, self.velocity_callback)
        
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.loginfo("Battery Monitor initialized")

    def gps_callback(self, msg):
        self.current_altitude = msg.altitude

    def velocity_callback(self, msg):
        self.current_speed = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)**0.5

    def update_battery(self):
        altitude_drain = self.current_altitude * self.altitude_drain_factor
        speed_drain = self.current_speed * self.speed_drain_factor
        total_drain = self.base_drain_rate + altitude_drain + speed_drain
        
        self.battery_level = max(0.0, self.battery_level - total_drain)
        self.battery_pub.publish(self.battery_level)
        
        if self.battery_level <= self.critical_battery_threshold:
            self.battery_status_pub.publish("CRITICAL")
        elif self.battery_level <= self.low_battery_threshold:
            self.battery_status_pub.publish("LOW")
        else:
            self.battery_status_pub.publish("NORMAL")

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
