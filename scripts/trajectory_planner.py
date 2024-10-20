import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

class TrajectoryPlanner:
    def __init__(self):
        rospy.init_node('trajectory_planner')
        
        # Parametri
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.hover_altitude = rospy.get_param('~hover_altitude', 2.0)
        
        # Stare
        self.current_pose = None
        self.target_pose = None
        self.path = None
        
        # Publishers
        self.path_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/target_pose', PoseStamped, self.target_pose_callback)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
    def pose_callback(self, msg):
        self.current_pose = msg

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def update(self):
        if self.current_pose is None or self.target_pose is None:
            return
        
        # Calculăm traseul
        self.path = Path()
        self.path.header = self.current_pose.header
        self.path.poses.append(self.current_pose)
        self.path.poses.append(self.target_pose)
        
        # Publicăm traseul
        self.path_pub.publish(self.path)

    def run(self):
        while not rospy.is _shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    planner = TrajectoryPlanner()
    planner.run()
