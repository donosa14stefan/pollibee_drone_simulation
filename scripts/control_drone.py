import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Bool

class AdvancedObstacleAvoidance:
    def __init__(self):
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # metri
        self.field_of_view = rospy.get_param('~field_of_view', 180)  # grade
        self.angular_resolution = rospy.get_param('~angular_resolution', 1)  # grade

    def detect_obstacle(self, laser_scan):
        obstacle_directions = []
        angle = -self.field_of_view / 2
        for distance in laser_scan.ranges:
            if distance < self.safe_distance:
                obstacle_directions.append(angle)
            angle += self.angular_resolution
        return obstacle_directions

    def adjust_velocity(self, cmd_vel, obstacle_directions):
        if not obstacle_directions:
            return cmd_vel

        mean_direction = sum(obstacle_directions) / len(obstacle_directions)
        adjusted_cmd_vel = Twist()
        adjusted_cmd_vel.linear.x = cmd_vel.linear.x * 0.5
        adjusted_cmd_vel.angular.z = -mean_direction * 0.02

        return adjusted_cmd_vel

class TrajectoryPlanner:
    def __init__(self):
        self.grid_resolution = rospy.get_param('~grid_resolution', 0.1)  # metri
        self.field_size = rospy.get_param('~field_size', [10, 10])  # metri

    def plan_trajectory(self, start, goal, obstacles):
        grid = self.create_grid(obstacles)
        path = self.a_star(grid, start, goal)
        return self.smooth_path(path)

    def create_grid(self, obstacles):
        grid = np.zeros((int(self.field_size[0] / self.grid_resolution),
                         int(self.field_size[1] / self.grid_resolution)))
        for obstacle in obstacles:
            x = int(obstacle[0] / self.grid_resolution)
            y = int(obstacle[1] / self.grid_resolution)
            grid[x, y] = 1
        return grid

    def a_star(self, grid, start, goal):
        # Implementare simplificată a algoritmului A*
        # Această funcție ar trebui să returneze o cale optimă între start și goal
        # evitând obstacolele din grid
        pass

    def smooth_path(self, path):
        # Implementare a unui algoritm de netezire a traiectoriei
        # Această funcție ar trebui să ia calea brută și să o transforme într-o traiectorie mai lină
        pass

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Parametri
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.1)
        self.battery_capacity = rospy.get_param('~battery_capacity', 100.0)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)
        self.full_battery_threshold = rospy.get_param('~full_battery_threshold', 95.0)
        
        # Componente
        self.obstacle_avoidance = AdvancedObstacleAvoidance()
        self.trajectory_planner = TrajectoryPlanner()
        
        # State
        self.current_pose = None
        self.target_pose = None
        self.battery_level = self.battery_capacity
        self.is_charging = False
        self.obstacles = []
        self.current_trajectory = None
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.battery_pub = rospy.Publisher('battery_status', Float32, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('planned_path', Path, self.path_callback)
        rospy.Subscriber('laser_scan', LaserScan, self.laser_callback)
        rospy.Subscriber('obstacles', PointCloud, self.obstacle_callback)
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def path_callback(self, msg):
        if len(msg.poses) > 0:
            self.target_pose = msg.poses[-1].pose
            self.plan_trajectory()
        
    def laser_callback(self, msg):
        obstacle_directions = self.obstacle_avoidance.detect_obstacle(msg)
        if obstacle_directions:
            self.adjust_trajectory(obstacle_directions)
        
    def obstacle_callback(self, msg):
        self.obstacles = [(point.x, point.y, point.z) for point in msg.points]
        self.plan_trajectory()
        
    def plan_trajectory(self):
        if self.current_pose and self.target_pose:
            start = (self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)
            goal = (self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z)
             self.current_trajectory = self.trajectory_planner.plan_trajectory(start, goal, self.obstacles)
def initialize_mission(self):
        self.mission_waypoints = self.generate_survey_pattern()
        self.current_waypoint_index = 0
        self.set_next_waypoint()

    def generate_survey_pattern(self):
        # Generează un model de survol în zigzag pentru acoperirea întregului câmp
        field_length = rospy.get_param('~field_length', 100)
        field_width = rospy.get_param('~field_width', 50)
        survey_altitude = rospy.get_param('~survey_altitude', 10)
        spacing = rospy.get_param('~survey_spacing', 5)

        waypoints = []
        for x in np.arange(0, field_length, spacing):
            y = 0 if len(waypoints) % 2 == 0 else field_width
            waypoints.append(Point(x, y, survey_altitude))
            y = field_width if len(waypoints) % 2 == 0 else 0
            waypoints.append(Point(x, y, survey_altitude))

        return waypoints

    def set_next_waypoint(self):
        if self.current_waypoint_index < len(self.mission_waypoints):
            self.target_pose = PoseStamped()
            self.target_pose.pose.position = self.mission_waypoints[self.current_waypoint_index]
            self.current_waypoint_index += 1
            self.plan_trajectory()
        else:
            rospy.loginfo("Mission complete")
            self.land()

    def land(self):
        land_cmd = Twist()
        land_cmd.linear.z = -0.5  # Viteză de coborâre moderată
        while self.current_pose.position.z > 0.1:  # Presupunem că 0.1m este înălțimea de aterizare
            self.cmd_vel_pub.publish(land_cmd)
            rospy.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Oprește toate motoarele
        rospy.loginfo("Landed successfully")

    def take_off(self, target_altitude):
        takeoff_cmd = Twist()
        takeoff_cmd.linear.z = 0.5  # Viteză de urcare moderată
        while self.current_pose.position.z < target_altitude:
            self.cmd_vel_pub.publish(takeoff_cmd)
            rospy.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Stabilizează drona la altitudinea dorită
        rospy.loginfo(f"Reached target altitude of {target_altitude}m")

    def adjust_for_wind(self, wind_speed, wind_direction):
        # Ajustează comanda de viteză pentru a compensa efectul vântului
        wind_compensation = Twist()
        wind_compensation.linear.x = -wind_speed * np.cos(wind_direction)
        wind_compensation.linear.y = -wind_speed * np.sin(wind_direction)
        return wind_compensation

    def detect_and_approach_flower(self, flower_position):
        # Implementează logica pentru detectarea și apropierea de o floare
        approach_cmd = Twist()
        distance_to_flower = np.linalg.norm([
            flower_position.x - self.current_pose.position.x,
            flower_position.y - self.current_pose.position.y,
            flower_position.z - self.current_pose.position.z
        ])
        
        while distance_to_flower > 0.5:  # Distanța de apropiere dorită
            # Calculează direcția către floare
            direction = np.array([
                flower_position.x - self.current_pose.position.x,
                flower_position.y - self.current_pose.position.y,
                flower_position.z - self.current_pose.position.z
            ])
            direction = direction / np.linalg.norm(direction)
            
            # Setează comanda de viteză
            approach_cmd.linear.x = direction[0] * 0.2  # Viteză redusă pentru apropiere precisă
            approach_cmd.linear.y = direction[1] * 0.2
            approach_cmd.linear.z = direction[2] * 0.2
            
            self.cmd_vel_pub.publish(approach_cmd)
            rospy.sleep(0.1)
            
            # Actualizează distanța
            distance_to_flower = np.linalg.norm([
                flower_position.x - self.current_pose.position.x,
                flower_position.y - self.current_pose.position.y,
                flower_position.z - self.current_pose.position.z
            ])
        
        self.cmd_vel_pub.publish(Twist())  # Oprește mișcarea
        rospy.loginfo("Reached flower position")

    def pollinate_flower(self):
        # Implementează logica pentru polenizarea florii
        rospy.loginfo("Starting pollination process")
        
        # Activează mecanismul de polenizare
        self.activate_pollination_mechanism()
        
        # Așteaptă finalizarea procesului de polenizare
        rospy.sleep(2)  # Timpul necesar pentru polenizare
        
        # Dezactivează mecanismul de polenizare
        self.deactivate_pollination_mechanism()
        
        rospy.loginfo("Pollination complete")

    def activate_pollination_mechanism(self):
        # Implementează activarea mecanismului de polenizare
        activation_msg = Bool()
        activation_msg.data = True
        self.pollination_mechanism_pub.publish(activation_msg)

    def deactivate_pollination_mechanism(self):
        # Implementează dezactivarea mecanismului de polenizare
        activation_msg = Bool()
        activation_msg.data = False
        self.pollination_mechanism_pub.publish(activation_msg)

    def process_image(self, image_msg):
        # Procesează imaginea pentru a detecta florile
        # Această funcție ar trebui să utilizeze algoritmul YOLO sau alt algoritm de detecție
        # Returnează o listă de poziții ale florilor detectate
        # Implementarea exactă depinde de algoritmul de detecție ales
        pass

    def image_callback(self, image_msg):
        flower_positions = self.process_image(image_msg)
        for flower_position in flower_positions:
            self.detect_and_approach_flower(flower_position)
            self.pollinate_flower()
            self.return_to_survey_altitude()

    def return_to_survey_altitude(self):
        survey_altitude = rospy.get_param('~survey_altitude', 10)
        self.take_off(survey_altitude)

        def run(self):
        self.initialize_mission()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.battery_level <= self.low_battery_threshold:
                self.return_to_charging_station()
            elif self.is_charging and self.battery_level >= self.full_battery_threshold:
                self.resume_mission()
            elif not self.is_charging:
                if self.has_reached_target():
                    rospy.loginfo("Reached target position")
                    self.set_next_waypoint()
                elif self.target_pose is not None:
                    cmd_vel = self.calculate_velocity()
                    adjusted_cmd_vel = self.obstacle_avoidance.adjust_velocity(cmd_vel, self.obstacles)
                    wind_compensation = self.adjust_for_wind(self.wind_speed, self.wind_direction)
                    final_cmd_vel = self.combine_velocities(adjusted_cmd_vel, wind_compensation)
                    self.cmd_vel_pub.publish(final_cmd_vel)

            self.update_battery()
            self.publish_status()
            rate.sleep()

    def combine_velocities(self, vel1, vel2):
        combined_vel = Twist()
        combined_vel.linear.x = vel1.linear.x + vel2.linear.x
        combined_vel.linear.y = vel1.linear.y + vel2.linear.y
        combined_vel.linear.z = vel1.linear.z + vel2.linear.z
        combined_vel.angular.z = vel1.angular.z + vel2.angular.z
        return combined_vel

    def publish_status(self):
        status_msg = DroneStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.position = self.current_pose.position
        status_msg.battery_level = self.battery_level
        status_msg.is_charging = self.is_charging
        status_msg.current_waypoint = self.current_waypoint_index
        status_msg.total_waypoints = len(self.mission_waypoints)
        self.status_pub.publish(status_msg)

    def wind_callback(self, wind_msg):
        self.wind_speed = wind_msg.speed
        self.wind_direction = wind_msg.direction

    def obstacle_callback(self, obstacle_msg):
        self.obstacles = obstacle_msg.points

    def emergency_land(self):
        rospy.logwarn("Emergency landing initiated")
        land_cmd = Twist()
        land_cmd.linear.z = -1.0  # Viteză de coborâre rapidă pentru aterizare de urgență
        while self.current_pose.position.z > 0.1:
            self.cmd_vel_pub.publish(land_cmd)
            rospy.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Oprește toate motoarele
        rospy.logwarn("Emergency landing completed")

    def resume_mission(self):
        self.is_charging = False
        rospy.loginfo("Resuming mission")
        self.take_off(rospy.get_param('~survey_altitude', 10))
        self.set_next_waypoint()

       def __init__(self):
        rospy.init_node('drone_controller')

        # Parametri
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.1)
        self.battery_capacity = rospy.get_param('~battery_capacity', 100.0)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)
        self.full_battery_threshold = rospy.get_param('~full_battery_threshold', 90.0)
        self.charging_rate = rospy.get_param('~charging_rate', 1.0)  # % per second
        self.discharging_rate = rospy.get_param('~discharging_rate', 0.1)  # % per second

        # Stare
        self.current_pose = PoseStamped()
        self.target_pose = None
        self.battery_level = 100.0
        self.is_charging = False
        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.obstacles = []

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('drone_status', DroneStatus, queue_size=10)
        self.pollination_mechanism_pub = rospy.Publisher('pollination_mechanism', Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('drone_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('wind', WindInfo, self.wind_callback)
        rospy.Subscriber('obstacles', PointCloud, self.obstacle_callback)
        rospy.Subscriber('camera/image_raw', Image, self.image_callback)

        # Servicii
        self.emergency_land_service = rospy.Service('emergency_land', EmergencyLand, self.handle_emergency_land)

        # Inițializare componente auxiliare
        self.obstacle_avoidance = ObstacleAvoidance()
        self.path_planner = PathPlanner()

        rospy.loginfo("Drone Controller initialized")

    def pose_callback(self, msg):
        self.current_pose = msg

    def handle_emergency_land(self, req):
        self.emergency_land()
        return EmergencyLandResponse(success=True, message="Emergency landing completed")

    def update_battery(self):
        if self.is_charging:
            self.battery_level = min(100.0, self.battery_level + self.charging_rate * 0.1)  # 0.1 pentru că rulăm la 10 Hz
        else:
            self.battery_level = max(0.0, self.battery_level - self.discharging_rate * 0.1)

    def return_to_charging_station(self):
        rospy.loginfo("Returning to charging station")
        charging_station_pose = self.get_charging_station_pose()
        self.target_pose = charging_station_pose
        self.fly_to_target()
        self.land()
        self.start_charging()

    def get_charging_station_pose(self):
        # În practică, aceasta ar putea fi o poziție fixă sau ar putea fi obținută dintr-un serviciu
        return PoseStamped(
            header=Header(frame_id="map"),
            pose=Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        )

    def start_charging(self):
        self.is_charging = True
        rospy.loginfo("Started charging")

        def fly_to_target(self):
        while not self.has_reached_target():
            cmd_vel = self.calculate_velocity()
            adjusted_cmd_vel = self.obstacle_avoidance.adjust_velocity(cmd_vel, self.obstacles)
            wind_compensation = self.adjust_for_wind(self.wind_speed, self.wind_direction)
            final_cmd_vel = self.combine_velocities(adjusted_cmd_vel, wind_compensation)
            self.cmd_vel_pub.publish(final_cmd_vel)
            rospy.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Oprește mișcarea când ajunge la țintă

    def calculate_velocity(self):
        if self.current_pose is None or self.target_pose is None:
            return Twist()

        # Calculează eroarea de poziție
        position_error = np.array([
            self.target_pose.position.x - self.current_pose.position.x,
            self.target_pose.position.y - self.current_pose.position.y,
            self.target_pose.position.z - self.current_pose.position.z
        ])

        # Calculează viteza liniară
        linear_velocity = np.clip(position_error, -self.max_velocity, self.max_velocity)

        # Creează mesajul Twist
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity[0]
        cmd_vel.linear.y = linear_velocity[1]
        cmd_vel.linear.z = linear_velocity[2]

        return cmd_vel

    def adjust_for_wind(self, wind_speed, wind_direction):
        # Implementează compensarea vântului
        wind_compensation = Twist()
        wind_compensation.linear.x = -wind_speed * np.cos(wind_direction)
        wind_compensation.linear.y = -wind_speed * np.sin(wind_direction)
        return wind_compensation

    def has_reached_target(self):
        if self.current_pose is None or self.target_pose is None:
            return False

        position_error = np.linalg.norm([
            self.target_pose.position.x - self.current_pose.position.x,
            self.target_pose.position.y - self.current_pose.position.y,
            self.target_pose.position.z - self.current_pose.position.z
        ])

        return position_error < self.position_tolerance

    def land(self):
        rospy.loginfo("Landing initiated")
        land_cmd = Twist()
        land_cmd.linear.z = -0.5  # Viteză de coborâre moderată pentru aterizare normală
        while self.current_pose.position.z > 0.1:
            self.cmd_vel_pub.publish(land_cmd)
            rospy.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Oprește toate motoarele
        rospy.loginfo("Landing completed")

    def take_off(self, target_altitude):
        rospy.loginfo(f"Taking off to altitude: {target_altitude}")
        take_off_cmd = Twist()
        take_off_cmd.linear.z = 0.5  # Viteză de urcare moderată
        while self.current_pose.position.z < target_altitude:
            self.cmd_vel_pub.publish(take_off_cmd)
            rospy.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Oprește urcarea
        rospy.loginfo("Take off completed")

        def set_next_waypoint(self):
        next_waypoint = self.path_planner.get_next_waypoint()
        if next_waypoint:
            self.target_pose = next_waypoint
            rospy.loginfo(f"Setting next waypoint: {self.target_pose.position}")
        else:
            rospy.loginfo("Mission completed. Returning to home.")
            self.return_to_home()

    def return_to_home(self):
        home_pose = self.get_home_pose()
        self.target_pose = home_pose
        self.fly_to_target()
        self.land()
        rospy.loginfo("Returned to home. Mission completed.")

    def get_home_pose(self):
        # În practică, aceasta ar putea fi o poziție fixă sau ar putea fi obținută dintr-un serviciu
        return PoseStamped(
            header=Header(frame_id="map"),
            pose=Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        )

    def image_callback(self, msg):
        # Procesează imaginea pentru detecția florilor
        flowers = self.detect_flowers(msg)
        if flowers:
            self.update_path_with_flowers(flowers)

    def detect_flowers(self, image_msg):
        # Implementează algoritmul de detecție a florilor
        # Aceasta este o implementare simplificată
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # Adaugă aici logica de detecție a florilor
        # Returnează o listă de poziții ale florilor detectate
        return []

    def update_path_with_flowers(self, flowers):
        for flower in flowers:
            self.path_planner.add_waypoint(flower)
        rospy.loginfo(f"Added {len(flowers)} new flowers to the path")

    def activate_pollination_mechanism(self):
        self.pollination_mechanism_pub.publish(Bool(True))
        rospy.sleep(2)  # Activează mecanismul de polenizare pentru 2 secunde
        self.pollination_mechanism_pub.publish(Bool(False))

    def initialize_mission(self):
        rospy.loginfo("Initializing mission")
        self.take_off(rospy.get_param('~survey_altitude', 10))
        initial_waypoints = self.path_planner.generate_survey_path()
        for waypoint in initial_waypoints:
            self.path_planner.add_waypoint(waypoint)
        self.set_next_waypoint()

    def execute_mission(self):
        while not rospy.is_shutdown():
            if self.battery_level <= self.low_battery_threshold:
                self.return_to_charging_station()
            elif self.is_charging and self.battery_level >= self.full_battery_threshold:
                self.resume_mission()
            elif not self.is_charging:
                if self.has_reached_target():
                    rospy.loginfo("Reached target position")
                    self.activate_pollination_mechanism()
                    self.set_next_waypoint()
                elif self.target_pose is not None:
                    self.fly_to_target()

            self.update_battery()
            self.publish_status()
            rospy.sleep(0.1)

    def resume_mission(self):
        self.is_charging = False
        rospy.loginfo("Resuming mission")
        self.take_off(rospy.get_param('~survey_altitude', 10))
        self.set_next_waypoint()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.initialize_mission()
        controller.execute_mission
        def publish_status(self):
        status_msg = DroneStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.position = self.current_pose.pose.position
        status_msg.battery_level = self.battery_level
        status_msg.is_charging = self.is_charging
        status_msg.wind_speed = self.wind_speed
        status_msg.wind_direction = self.wind_direction
        self.status_pub.publish(status_msg)

    def wind_callback(self, msg):
        self.wind_speed = msg.speed
        self.wind_direction = msg.direction

    def obstacle_callback(self, msg):
        self.obstacles = msg.points

    def emergency_land(self):
        rospy.logwarn("Emergency landing initiated!")
        while self.current_pose.position.z > 0.1:
            land_cmd = Twist()
            land_cmd.linear.z = -1.0  # Viteză de coborâre rapidă pentru aterizare de urgență
            self.cmd_vel_pub.publish(land_cmd)
            rospy.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())  # Oprește toate motoarele
        rospy.logwarn("Emergency landing completed")

    def combine_velocities(self, cmd_vel, wind_compensation):
        final_cmd_vel = Twist()
        final_cmd_vel.linear.x = cmd_vel.linear.x + wind_compensation.linear.x
        final_cmd_vel.linear.y = cmd_vel.linear.y + wind_compensation.linear.y
        final_cmd_vel.linear.z = cmd_vel.linear.z + wind_compensation.linear.z
        
        # Limitarea vitezei maxime
        magnitude = np.linalg.norm([final_cmd_vel.linear.x, final_cmd_vel.linear.y, final_cmd_vel.linear.z])
        if magnitude > self.max_velocity:
            scale = self.max_velocity / magnitude
            final_cmd_vel.linear.x *= scale
            final_cmd_vel.linear.y *= scale
            final_cmd_vel.linear.z *= scale

        return final_cmd_vel

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.initialize_mission()
        controller.execute_mission()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {str(e)}")
        rospy.logerr(traceback.format_exc())
        controller.emergency_land()
    finally:
        rospy.loginfo("Drone controller shutting down.")
    
    def adjust_trajectory(self, obstacle_directions):
        if self.current_trajectory:
            # Implementați logica pentru ajustarea traiectoriei în funcție de obstacolele detectate
            pass
        
        def calculate_velocity(self):
        if self.current_pose is None or self.target_pose is None:
            return Twist()

        # Calculăm eroarea de poziție
        position_error = np.array([
            self.target_pose.position.x - self.current_pose.position.x,
            self.target_pose.position.y - self.current_pose.position.y,
            self.target_pose.position.z - self.current_pose.position.z
        ])

        # Calculăm viteza lineară
        linear_velocity = np.clip(position_error, -self.max_velocity, self.max_velocity)

        # Creăm mesajul Twist
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity[0]
        cmd_vel.linear.y = linear_velocity[1]
        cmd_vel.linear.z = linear_velocity[2]

        return cmd_vel

    def update_battery(self):
        if self.is_charging:
            self.battery_level = min(self.battery_level + 0.1, self.battery_capacity)
        else:
            self.battery_level = max(self.battery_level - 0.05, 0)

        self.battery_pub.publish(self.battery_level)

        if self.battery_level <= self.low_battery_threshold and not self.is_charging:
            self.return_to_charging_station()
        elif self.battery_level >= self.full_battery_threshold and self.is_charging:
            self.resume_mission()

    def return_to_charging_station(self):
        # Implementați logica pentru întoarcerea la stația de încărcare
        self.is_charging = True
        rospy.loginfo("Returning to charging station")

    def resume_mission(self):
        # Implementați logica pentru reluarea misiunii după încărcare
        self.is_charging = False
        rospy.loginfo("Resuming mission")

    def has_reached_target(self):
        if self.current_pose is None or self.target_pose is None:
            return False

        position_error = np.linalg.norm([
            self.target_pose.position.x - self.current_pose.position.x,
            self.target_pose.position.y - self.current_pose.position.y,
            self.target_pose.position.z - self.current_pose.position.z
        ])

        return position_error < self.position_tolerance

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self.is_charging:
                if self.has_reached_target():
                    rospy.loginfo("Reached target position")
                    self.target_pose = None
                elif self.target_pose is not None:
                    cmd_vel = self.calculate_velocity()
                    adjusted_cmd_vel = self.obstacle_avoidance.adjust_velocity(cmd_vel, self.obstacles)
                    self.cmd_vel_pub.publish(adjusted_cmd_vel)

            self.update_battery()
            rate.sleep()

def main():
    controller = DroneController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
