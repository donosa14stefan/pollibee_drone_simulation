#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist

class WindSimulator:
    def __init__(self):
        self.wind_pub = rospy.Publisher('wind', Twist, queue_size=10)
        
        self.current_wind = Twist()
        self.wind_speed = 0.5
        self.wind_direction = 0.0
        
        rospy.loginfo("Wind Simulator initialized")

    def update(self):
        # Update wind speed and direction
        self.current_wind.linear.x = self.wind_speed * np.cos(self.wind_direction)
        self.current_wind.linear.y = self.wind_speed * np.sin(self.wind_direction)
        
        # Publish wind
        self.wind_pub.publish(self.current_wind)

if __name__ == '__main__':
    try:
        simulator = WindSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
