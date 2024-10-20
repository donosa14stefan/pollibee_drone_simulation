import rospy
import numpy as np
from geometry_msgs.msg import Twist

class WindSimulator:
    def __init__(self):
        rospy.init_node('wind_simulator')
        
        # Parametri
        self.wind_speed = rospy.get_param('~wind_speed', 0.5)
        self.wind_direction = rospy.get_param('~wind_direction', 0.0)
        
        # Publishers
        self.wind_pub = rospy.Publisher('/wind', Twist, queue_size=10)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
    def update(self):
        # Calculăm viteza vântului
        wind = Twist()
        wind.linear.x = self.wind_speed * np.cos(self.wind_direction)
        wind.linear.y = self.wind_speed * np.sin(self.wind_direction)
        
        # Publicăm viteza vântului
        self.wind_pub.publish(wind)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    simulator = WindSimulator()
    simulator.run()
