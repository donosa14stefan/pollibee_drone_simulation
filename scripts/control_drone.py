#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Parameters
        self.max_speed = rospy.get_param('~max_speed', 0.5)
        self.min_distance = rospy.get_param('~min_distance', 1.0)
        
        # State variables
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])
        self.plant_detections = []
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('plant_detections', Float32MultiArray, self.detection_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Drone Controller initialized")

    def odom_callback(self, msg):
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.orientation = np.array(euler_from_quaternion(orientation_list))

    def detection_callback(self, msg):
        self.plant_detections = np.array(msg.data).reshape(-1, 5)  # [x, y, w, h, confidence]

    def get_closest_plant(self):
        if len(self.plant_detections) == 0:
            return None
        
        plant_centers = self.plant_detections[:, :2] + self.plant_detections[:, 2:4] / 2
        distances = np.linalg.norm(plant_centers - self.position[:2], axis=1)
        closest_idx = np.argmin(distances)
        
        return self.plant_detections[closest_idx]

    def move_to_plant(self, plant):
        target = np.array([plant[0] + plant[2]/2, plant[1] + plant[3]/2, self.position[2]])
        direction = target - self.position
        distance = np.linalg.norm(direction)
        
        if distance < self.min_distance:
            rospy.loginfo("Reached plant. Simulating pollination.")
            rospy.sleep(2)  # Simulate pollination time
            return
        
        # Normalize direction and apply max speed
        velocity = direction / distance * self.max_speed
        
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity[0]
        cmd_vel.linear.y = velocity[1]
        cmd_vel.linear.z = velocity[2]
        
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            closest_plant = self.get_closest_plant()
            
            if closest_plant is not None:
                               self.move_to_plant(closest_plant)
            else:
                # If no plant is detected, perform a search pattern
                self.search_pattern()
            
            self.rate.sleep()

    def search_pattern(self):
        # Implement a simple search pattern
        cmd_vel = Twist()
        cmd_vel.linear.x = self.max_speed * np.cos(self.orientation[2])
        cmd_vel.linear.y = self.max_speed * np.sin(self.orientation[2])
        cmd_vel.angular.z = 0.1  # Slow rotation to scan the area
        
        self.cmd_vel_pub.publish(cmd_vel)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class DroneController:
    def __init__(self):
        # ... (codul existent)
        self.flower_sub = rospy.Subscriber('flower_detections', PoseStamped, self.flower_callback)
        self.pollinate_pub = rospy.Publisher('pollinate', Bool, queue_size=10)
        self.current_target = None

    def flower_callback(self, msg):
        self.current_target = msg.pose

    def run(self):
        while not rospy.is_shutdown():
            if self.current_target:
                # Calculați comanda de viteză pentru a se deplasa către floare
                cmd_vel = self.calculate_velocity_command(self.current_target)
                self.cmd_vel_pub.publish(cmd_vel)

                # Verificați dacă drona este suficient de aproape de floare pentru polenizare
                if self.is_close_to_target(self.current_target):
                    self.pollinate()

            self.rate.sleep()

    def calculate_velocity_command(self, target):
        # Implementați logica pentru calculul comenzii de viteză
        # Aceasta ar trebui să ghideze drona către poziția țintă
        cmd_vel = Twist()
        # ... calculul vitezelor liniare și unghiulare
        return cmd_vel

    def is_close_to_target(self, target):
        # Verificați dacă drona este suficient de aproape de țintă
        # Returnați True dacă este aproape, False în caz contrar
        return False  # Implementați logica proprie

    def pollinate(self):
        pollinate_msg = Bool()
        pollinate_msg.data = True
        self.pollinate_pub.publish(pollinate_msg)
        rospy.loginfo("Pollinating flower!")

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
