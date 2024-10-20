import rospy
import numpy as np
from sensor_msgs.msg import Illuminance

class LightSimulator:
    def __init__(self):
        rospy.init_node('light_simulator')
        
        # Parametri
        self.illuminance = rospy.get_param('~illuminance', 500.0)
        
        # Publishers
        self.illuminance_pub = rospy.Publisher('/illuminance', Illuminance, queue_size=10)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
    def update(self):
        # Publicăm iluminarea
        illuminance = Illuminance()
        illuminance.illuminance = self.illuminance
        self.illuminance_pub.publish(illuminance)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    simulator = LightSimulator()
    simulator.run()
