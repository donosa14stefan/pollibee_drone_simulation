import rospy
import numpy as np
from sensor_msgs.msg import CO2

class CO2Simulator:
    def __init__(self):
        rospy.init_node('co2_simulator')
        
        # Parametri
        self.co2_level = rospy.get_param('~co2_level', 400.0)
        
        # Publishers
        self.co2_pub = rospy.Publisher('/co2', CO2, queue_size=10)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
    def update(self):
        # Publicăm nivelul de CO2
        co2 = CO2()
        co2.co2 = self.co2_level
        self.co2_pub.publish(co2)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    simulator = CO2Simulator()
    simulator.run()
