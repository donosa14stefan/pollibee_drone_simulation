import rospy
import numpy as np
from sensor_msgs.msg import BatteryState

class BatterySimulator:
    def __init__(self):
        rospy.init_node('battery_simulator')
        
        # Parametri
        self.battery_capacity = rospy.get_param('~battery_capacity', 100.0)
        self.discharge_rate = rospy.get_param('~discharge_rate', 0.1)
        
        # Stare
        self.battery_level = self.battery_capacity
        
        # Publishers
        self.battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
    def update(self):
        # Calculăm nivelul bateriei
        self.battery_level -= self.discharge_rate
        
        # Publicăm nivelul bateriei
        battery_state = BatteryState()
        battery_state.percentage = self.battery_level
        self.battery_pub.publish(battery_state)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    simulator = BatterySimulator()
    simulator.run()
