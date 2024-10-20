import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Humidity

class EnvironmentSimulator:
    def __init__(self):
        rospy.init_node('environment_simulator')
        
        # Parametri
        self.temperature = rospy.get_param('~temperature', 25.0)
        self.humidity = rospy.get_param('~humidity', 60.0)
        
        # Publishers
        self.temperature_pub = rospy.Publisher('/temperature', Temperature, queue_size=10)
        self.humidity_pub = rospy.Publisher('/humidity', Humidity, queue_size=10)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
    def update(self):
        # Publicăm temperatura și umiditatea
        temperature = Temperature()
        temperature.temperature = self.temperature
        self.temperature_pub.publish(temperature)
        
        humidity = Humidity()
        humidity.relative_humidity = self.humidity
        self.humidity_pub.publish(humidity)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    simulator = EnvironmentSimulator()
    simulator.run()
