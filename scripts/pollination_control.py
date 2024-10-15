#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class PollinationController:
    def __init__(self):
        rospy.init_node('pollination_controller')
        
        self.flower_sub = rospy.Subscriber("flower_positions", PoseStamped, self.flower_callback)
        self.drone_pose_sub = rospy.Subscriber("drone_pose", PoseStamped, self.drone_callback)
        self.pollinate_pub = rospy.Publisher("pollinate", Bool, queue_size=10)
        
        self.flower_position = None
        self.drone_position = None
        self.pollination_distance = rospy.get_param('~pollination_distance', 0.5)  # meters
        
        rospy.loginfo("Pollination Controller Node Initialized")

    def flower_callback(self, msg):
        self.flower_position = msg.pose.position

    def drone_callback(self, msg):
        self.drone_position = msg.pose.position
        self.check_pollination()

    def check_pollination(self):
        if self.flower_position is None or self.drone_position is None:
            return
        
        distance = ((self.flower_position.x - self.drone_position.x)**2 +
                    (self.flower_position.y - self.drone_position.y)**2 +
                                        (self.flower_position.z - self.drone_position.z)**2)**0.5

        if distance <= self.pollination_distance:
            self.pollinate()

    def pollinate(self):
        pollinate_msg = Bool()
        pollinate_msg.data = True
        self.pollinate_pub.publish(pollinate_msg)
        rospy.loginfo("Pollinating flower!")

if __name__ == '__main__':
    try:
        controller = PollinationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
