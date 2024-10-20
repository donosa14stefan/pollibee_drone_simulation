import rospy
import numpy as np
from std_msgs.msg import Bool

class PollinationSimulator:
    def __init__(self):
        rospy.init_node('pollination_simulator')
        
        # Parametri
        self.pollination_rate = rospy.get_param('~pollination_rate', 0.1)
        
        # Stare
        self.is_pollinating = False
        
        # Publishers
        self.pollination_pub = rospy.Publisher('/pollination_active', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/pollination_command', Bool, self.pollination_callback)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
    def pollination_callback(self, msg):
        self.is_pollinating = msg.data
        
    def update(self):
        if self.is_pollinating:
            self.pollination_pub.publish(Bool(True))
        else:
            self.pollination_pub.publish(Bool(False))

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    simulator = PollinationSimulator()
    simulator.run()
