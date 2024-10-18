#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from mission import Mission
from obstacle_avoidance import ObstacleAvoidance
from flower_tracking import FlowerTracking
from battery_management import BatteryManagement

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Inițializare componente
        self.obstacle_avoidance = ObstacleAvoidance()
        self.flower_tracking = FlowerTracking()
        self.battery_management = BatteryManagement()
        
        # Parametri
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)  # rad/s
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)  # m
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)  # %
        self.full_battery_threshold = rospy.get_param('~full_battery_threshold', 90.0)  # %
        self.pollination_distance = rospy.get_param('~pollination_distance', 0.2)  # m
        
        # Stare curentă
        self.current_pose = PoseStamped()
        self.current_goal = PoseStamped()
        self.battery_level = 100.0
        self.is_charging = False
        self.obstacle_detected = False
        self.tracking_flower = False
        self.flower_position = None
        self.pollinated_flowers = 0
        
        # Misiuni și path
        self.missions = []
        self.current_mission = None
        self.mission_waypoint_index = 0
        self.current_path = Path()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('drone_status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('battery_state', Float32, self.battery_callback)
        rospy.Subscriber('scan', LaserScan, self.obstacle_detection_callback)
        rospy.Subscriber('flower_position', PoseStamped, self.flower_detection_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        
        # Logging
        self.log_file = open('drone_log.txt', 'w')
        self.log_event("Drone controller initialized")

    def pose_callback(self, msg):
        self.current_pose = msg
        self.check_goal_reached()

    def battery_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level < self.low_battery_threshold and not self.is_charging:
            self.log_event("Low battery. Returning to charging station.")
            self.return_to_charging_station()

    def obstacle_detection_callback(self, msg):
        self.obstacle_detected = self.obstacle_avoidance.detect_obstacle(msg)
        if self.obstacle_detected:
            self.log_event("Obstacle detected. Adjusting trajectory.")

        def flower_detection_callback(self, msg):
        self.flower_position = msg.pose.position
        if not self.tracking_flower:
            self.tracking_flower = True
            self.log_event("Flower detected. Starting tracking.")

    def path_callback(self, msg):
        self.current_path = msg
        self.mission_waypoint_index = 0
        self.log_event("New path received. Starting navigation.")

    def add_mission(self, mission):
        self.missions.append(mission)
        self.log_event(f"New mission added: {mission.name}")

    def start_next_mission(self):
        if self.missions:
            self.current_mission = self.missions.pop(0)
            self.mission_waypoint_index = 0
            self.log_event(f"Starting mission: {self.current_mission.name}")
            self.set_next_mission_goal()
        else:
            self.log_event("All missions completed")
            self.return_to_home()

    def set_next_mission_goal(self):
        if self.mission_waypoint_index < len(self.current_mission.waypoints):
            next_waypoint = self.current_mission.waypoints[self.mission_waypoint_index]
            self.set_goal(next_waypoint)
            self.mission_waypoint_index += 1
        else:
            self.log_event(f"Mission completed: {self.current_mission.name}")
            self.start_next_mission()

    def set_goal(self, waypoint):
        self.current_goal.pose.position.x = waypoint[0]
        self.current_goal.pose.position.y = waypoint[1]
        self.current_goal.pose.position.z = waypoint[2]
        self.log_event(f"New goal set: ({waypoint[0]}, {waypoint[1]}, {waypoint[2]})")

    def check_goal_reached(self):
        if self.current_goal is None:
            return

        distance = np.linalg.norm([
            self.current_pose.pose.position.x - self.current_goal.pose.position.x,
            self.current_pose.pose.position.y - self.current_goal.pose.position.y,
            self.current_pose.pose.position.z - self.current_goal.pose.position.z
        ])

        if distance < self.goal_tolerance:
            self.log_event("Goal reached")
            self.execute_mission_task()
            self.set_next_mission_goal()

    def execute_mission_task(self):
        if self.current_mission and self.mission_waypoint_index > 0:
            current_waypoint = self.current_mission.waypoints[self.mission_waypoint_index - 1]
            task = self.current_mission.tasks.get(tuple(current_waypoint))
            if task:
                self.log_event(f"Executing task: {task}")
                if task == "scan":
                    self.perform_scan()
                elif task == "pollinate":
                    self.flower_tracking.start_pollination()

    def perform_scan(self):
        self.log_event("Performing area scan")
        # Aici ar trebui să implementăm logica pentru scanarea zonei
        # De exemplu, am putea roti drona la 360 de grade
        scan_duration = 10  # secunde
        angular_speed = 2 * np.pi / scan_duration  # rad/s
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < scan_duration:
            cmd_vel = Twist()
            cmd_vel.angular.z = angular_speed
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)
        
        self.log_event("Scan completed")

        def return_to_charging_station(self):
        # Aici ar trebui să implementăm logica pentru întoarcerea la stația de încărcare
        self.log_event("Returning to charging station")
        charging_station_position = rospy.get_param('~charging_station_position', [0, 0, 0])
        self.set_goal(charging_station_position)
        self.is_charging = True

    def return_to_home(self):
        self.log_event("Returning to home position")
        home_position = rospy.get_param('~home_position', [0, 0, 1])
        self.set_goal(home_position)

    def calculate_velocity(self):
        if self.current_goal is None:
            return Twist()

        direction = np.array([
            self.current_goal.pose.position.x - self.current_pose.pose.position.x,
            self.current_goal.pose.position.y - self.current_pose.pose.position.y,
            self.current_goal.pose.position.z - self.current_pose.pose.position.z
        ])

        distance = np.linalg.norm(direction)
        if distance > 0:
            direction = direction / distance

        cmd_vel = Twist()
        cmd_vel.linear.x = direction[0] * self.linear_speed
        cmd_vel.linear.y = direction[1] * self.linear_speed
        cmd_vel.linear.z = direction[2] * self.linear_speed

        return cmd_vel

    def adjust_for_obstacles(self, cmd_vel):
        if self.obstacle_detected:
            adjusted_cmd_vel = self.obstacle_avoidance.adjust_velocity(cmd_vel)
            return adjusted_cmd_vel
        return cmd_vel

    def track_flower(self):
        if self.flower_position is None:
            return Twist()

        direction = np.array([
            self.flower_position.x - self.current_pose.pose.position.x,
            self.flower_position.y - self.current_pose.pose.position.y,
            self.flower_position.z - self.current_pose.pose.position.z
        ])

        distance = np.linalg.norm(direction)
        if distance > 0:
            direction = direction / distance

        cmd_vel = Twist()
        cmd_vel.linear.x = direction[0] * self.linear_speed * 0.5  # Reducem viteza pentru precizie
        cmd_vel.linear.y = direction[1] * self.linear_speed * 0.5
        cmd_vel.linear.z = direction[2] * self.linear_speed * 0.5

        if distance < self.pollination_distance:
            self.start_pollination()

        return cmd_vel

    def start_pollination(self):
        self.log_event("Starting pollination process")
        # Aici ar trebui să implementăm logica pentru procesul de polenizare
        # De exemplu, am putea activa un mecanism de vibrație sau un ventilator
        rospy.sleep(2)  # Simulăm procesul de polenizare timp de 2 secunde
        self.pollinated_flowers += 1
        self.log_event(f"Pollination complete. Total flowers pollinated: {self.pollinated_flowers}")
        self.tracking_flower = False

    def update_status(self):
        status_msg = String()
        if self.is_charging:
            status_msg.data = "Charging"
        elif self.tracking_flower:
            status_msg.data = "Tracking flower"
        elif self.obstacle_detected:
            status_msg.data = "Avoiding obstacle"
        elif self.current_mission:
            status_msg.data = f"On mission: {self.current_mission.name}"
        else:
            status_msg.data = "Idle"

        self.status_pub.publish(status_msg)

        def log_event(self, event):
        timestamp = rospy.get_time()
        log_entry = f"{timestamp}: {event}\n"
        rospy.loginfo(log_entry)
        self.log_file.write(log_entry)
        self.log_file.flush()

    def check_battery(self):
        if self.battery_level < self.low_battery_threshold and not self.is_charging:
            self.log_event("Battery level critical. Returning to charging station.")
            self.return_to_charging_station()
        elif self.battery_level >= self.full_battery_threshold and self.is_charging:
            self.log_event("Battery fully charged. Resuming mission.")
            self.is_charging = False
            self.start_next_mission()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.check_battery()

            if self.is_charging:
                self.battery_management.charge()
            elif self.tracking_flower:
                cmd_vel = self.track_flower()
            else:
                cmd_vel = self.calculate_velocity()
                cmd_vel = self.adjust_for_obstacles(cmd_vel)

            self.cmd_vel_pub.publish(cmd_vel)
            self.update_status()
            rate.sleep()

    def shutdown(self):
        self.log_event("Shutting down drone controller")
        self.cmd_vel_pub.publish(Twist())  # Stop the drone
        self.log_file.close()

class Mission:
    def __init__(self, name, waypoints, tasks):
        self.name = name
        self.waypoints = waypoints
        self.tasks = tasks

class ObstacleAvoidance:
    def __init__(self):
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # metri

    def detect_obstacle(self, laser_scan):
        for distance in laser_scan.ranges:
            if distance < self.safe_distance:
                return True
        return False

    def adjust_velocity(self, cmd_vel):
        # Implementare simplă: reducem viteza și viram ușor la dreapta
        adjusted_cmd_vel = Twist()
        adjusted_cmd_vel.linear.x = cmd_vel.linear.x * 0.5
        adjusted_cmd_vel.angular.z = -0.5  # viraj la dreapta
        return adjusted_cmd_vel

class FlowerTracking:
    def __init__(self):
        self.tracking_duration = rospy.get_param('~tracking_duration', 10)  # secunde

    def start_pollination(self):
        rospy.loginfo("Starting pollination process")
        # Simulăm procesul de polenizare
        rospy.sleep(2)
        rospy.loginfo("Pollination complete")

class BatteryManagement:
    def __init__(self):
        self.charging_rate = rospy.get_param('~charging_rate', 1.0)  # % per secundă

    def charge(self):
        # Simulăm procesul de încărcare
        rospy.sleep(1)  # Așteptăm o secundă
        return self.charging_rate

def main():
    try:
        controller = DroneController()
        
        # Adăugăm câteva misiuni de exemplu
        mission1 = Mission("Scan Area 1", [(1, 1, 1), (2, 2, 1), (3, 3, 1)], {(2, 2, 1): "scan"})
        mission2 = Mission("Pollinate Area 2", [(4, 4, 1), (5, 5, 1)], {(5, 5, 1): "pollinate"})
        controller.add_mission(mission1)
        controller.add_mission(mission2)

        controller.start_next_mission()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.shutdown()

if __name__ == '__main__':
    main()
