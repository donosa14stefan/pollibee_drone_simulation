import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Parametri
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.hover_altitude = rospy.get_param('~hover_altitude', 2.0)
        self.battery_threshold = rospy.get_param('~battery_threshold', 20.0)
        
        # Stare
        self.current_pose = None
        self.target_pose = None
        self.battery_level = 100.0
        self.is_pollinating = False
        self.path = None
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pollination_pub = rospy.Publisher('/pollination_active', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/trajectory', Path, self.path_callback)
        rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        
        self.rate = rospy.Rate(20 )  # 20 Hz
        
    def pose_callback(self, msg):
        self.current_pose = msg

    def path_callback(self, msg):
        self.path = msg

    def battery_callback(self, msg):
        self.battery_level = msg.percentage

    def update(self):
        if self.current_pose is None or self.path is None:
            return
        
        # Verificăm dacă dronele este în stare de polenizare
        if self.is_pollinating:
            self.pollination_pub.publish(Bool(True))
        else:
            self.pollination_pub.publish(Bool(False))
        
        # Verificăm nivelul bateriei
        if self.battery_level < self.battery_threshold:
            rospy.logwarn("Low battery! Returning to base.")
            self.is_pollinating = False
            self.target_pose = PoseStamped()
            self.target_pose.header = self.current_pose.header
            self.target_pose.pose.position.x = 0
            self.target_pose.pose.position.y = 0
            self.target_pose.pose.position.z = self.hover_altitude
        else:
            # Calculăm poziția țintă
            if len(self.path.poses) > 0:
                self.target_pose = self.path.poses[0]
                self.path.poses.pop(0)
            else:
                rospy.loginfo("Path completed!")
                self.is_pollinating = False
                self.target_pose = PoseStamped()
                self.target_pose.header = self.current_pose.header
                self.target_pose.pose.position.x = 0
                self.target_pose.pose.position.y = 0
                self.target_pose.pose.position.z = self.hover_altitude
        
        # Calculăm viteza de comandă
        velocity = Twist()
        velocity.linear.x = self.max_velocity * (self.target_pose.pose.position.x - self.current_pose.pose.position.x)
        velocity.linear.y = self.max_velocity * (self.target_pose.pose.position.y - self.current_pose.pose.position.y)
        velocity.linear.z = self.max_velocity * (self.target_pose.pose.position.z - self.current_pose.pose.position.z)
        
        # Publicăm viteza de comandă
        self.cmd_vel_pub.publish(velocity)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    controller = DroneController()
    controller.run()
