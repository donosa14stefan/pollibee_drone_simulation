import rospy
from std_msgs.msg import Bool

class PollinationMotor:
    def __init__(self):
        rospy.init_node('pollination_motor')
        
        # Stare
        self.is_pollinating = False
        
        # Publishers
        self.pollination_pub = rospy.Publisher('/pollination_active', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/pollination_command', Bool, self.pollination_callback)
        
    def pollination_callback(self, msg):
        self.is_pollinating = msg.data
        
    def update(self):
        if self.is_pollinating:
            self.pollination_pub.publish(Bool(True))
        else:
            self.pollination_pub.publish(Bool(False))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    motor = PollinationMotor()
    motor.run()
