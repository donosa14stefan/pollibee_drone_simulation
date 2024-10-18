#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cryptography.fernet import Fernet
import hmac
import hashlib
import os

class CommunicationHandler:
    def __init__(self):
        rospy.init_node('communication_handler')
        
        # Encryption and integrity check setup
        self.encryption_key = Fernet.generate_key()
        self.fernet = Fernet(self.encryption_key)
        self.hmac_key = os.urandom(32)
        
                # Publishers
        self.status_pub = rospy.Publisher('drone_status_encrypted', String, queue_size=10)
        self.command_pub = rospy.Publisher('base_station_command', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('battery_level', Float32, self.battery_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('pollination_status', Bool, self.pollination_callback)
        rospy.Subscriber('base_station_input_encrypted', String, self.base_station_callback)
        
        # State variables
        self.current_pose = None
        self.battery_level = None
        self.current_path = None
        self.is_pollinating = False
        
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.loginfo("Communication Handler initialized")

    def encrypt_message(self, message):
        return self.fernet.encrypt(message.encode())

    def decrypt_message(self, encrypted_message):
        return self.fernet.decrypt(encrypted_message).decode()

    def create_hmac(self, message):
        h = hmac.new(self.hmac_key, message.encode(), hashlib.sha256)
        return h.hexdigest()

    def verify_hmac(self, message, received_hmac):
        calculated_hmac = self.create_hmac(message)
        return hmac.compare_digest(calculated_hmac, received_hmac)

    def send_encrypted_message(self, topic, message):
        encrypted_message = self.encrypt_message(message)
        hmac_value = self.create_hmac(message)
        self.status_pub.publish(f"{encrypted_message}|{hmac_value}")

    def receive_encrypted_message(self, encrypted_message_with_hmac):
        encrypted_message, received_hmac = encrypted_message_with_hmac.split('|')
        decrypted_message = self.decrypt_message(encrypted_message)
        if self.verify_hmac(decrypted_message, received_hmac):
            return decrypted_message
        else:
            rospy.logerr("Message integrity check failed!")
            return None

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def path_callback(self, msg):
        self.current_path = msg

    def pollination_callback(self, msg):
        self.is_pollinating = msg.data

    def base_station_callback(self, msg):
        encrypted_command = msg.data
        command = self.receive_encrypted_message(encrypted_command)
        if command:
            self.process_command(command)
        else:
            rospy.logwarn("Received invalid command")

    def process_command(self, command):
        if command == "START_MISSION":
            self.command_pub.publish("START")
        elif command == "ABORT_MISSION":
            self.command_pub.publish("ABORT")
        elif command == "RETURN_TO_BASE":
            self.command_pub.publish("RTB")
        else:
            rospy.logwarn(f"Unknown command received: {command}")

    def generate_status_message(self):
        status = {
            "position": {
                "x": self.current_pose.position.x if self.current_pose else None,
                "y": self.current_pose.position.y if self.current_pose else None,
                "z": self.current_pose.position.z if self.current_pose else None
            },
            "battery_level": self.battery_level,
            "waypoints_remaining": len(self.current_path.poses) if self.current_path else 0,
                        "is_pollinating": self.is_pollinating
        }
        return json.dumps(status)

    def run(self):
        while not rospy.is_shutdown():
            status_message = self.generate_status_message()
            self.send_encrypted_message('drone_status', status_message)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        handler = CommunicationHandler()
        handler.run()
    except rospy.ROSInterruptException:
        pass
