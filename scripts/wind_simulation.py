#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Wrench

class WindSimulation:
    def __init__(self):
        rospy.init_node('wind_simulation')
        
        # Parameters
        self.wind_mean = rospy.get_param('~wind_mean', [1.0, 0.0, 0.0])  # Mean wind vector [x, y, z]
        self.wind_variance = rospy.get_param('~wind_variance', 0.5)  # Variance of wind speed
        self.update_rate = rospy.get_param('~update_rate', 10)  # Hz
        
        # Publishers
        self.wind_force_pub = rospy.Publisher('wind_force', Wrench, queue_size=10)
        
        self.rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo("Wind Simulation Node Initialized")

    def generate_wind(self):
        # Generate random wind vector based on mean and variance
        wind_x = np.random.normal(self.wind_mean[0], self.wind_variance)
        wind_y = np.random.normal(self.wind_mean[1], self.wind_variance)
        wind_z = np.random.normal(self.wind_mean[2], self.wind_variance)
        
        return Vector3(wind_x, wind_y, wind_z)

    def run(self):
        while not rospy.is_shutdown():
            # Generate wind force
            wind_vector = self.generate_wind()
            
            # Create Wrench message
            wind_force = Wrench()
            wind_force.force = wind_vector
            
            # Publish wind force
            self.wind_force_pub.publish(wind_force)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        wind_sim = WindSimulation()
        wind_sim.run()
    except rospy.ROSInterruptException:
        pass
