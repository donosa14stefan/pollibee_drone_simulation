#!/usr/bin/env python3

import rospy
import sys
from control_drone import DroneController
from yolo_detection import YOLODetector
from pollination_motor import PollinationMotor
from trajectory_planner import TrajectoryPlanner
from communication_handler import CommunicationHandler
from data_logging import DataLogger
from battery_monitor import BatteryMonitor
from obstacle_avoidance import ObstacleAvoidance
from wind_simulation import WindSimulator
from mission_planner import MissionPlanner
from sensor_fusion import SensorFusion
from error_handling import ErrorHandler
from performance_metrics import PerformanceMetrics
from simulation_interface import SimulationInterface

class PolliBeeDroneSimulation:
    def __init__(self):
        rospy.init_node('pollibee_drone_simulation')
        self.error_handler = ErrorHandler()
        
        try:
            self.drone_controller = DroneController()
            self.yolo_detector = YOLODetector()
            self.pollination_motor = PollinationMotor()
            self.trajectory_planner = TrajectoryPlanner()
            self.communication_handler = CommunicationHandler()
            self.data_logger = DataLogger()
            self.battery_monitor = BatteryMonitor()
            self.obstacle_avoidance = ObstacleAvoidance()
            self.wind_simulator = WindSimulator()
            self.mission_planner = MissionPlanner()
            self.sensor_fusion = SensorFusion()
            self.performance_metrics = PerformanceMetrics()
            self.simulation_interface = SimulationInterface()
            
            rospy.loginfo("All modules initialized successfully")
        except Exception as e:
            self.error_handler.handle_error("Initialization Error", e)
            sys.exit(1)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                self.mission_planner.update()
                self.drone_controller.update()
                self.yolo_detector.detect()
                self.pollination_motor.update()
                self.trajectory_planner.update()
                self.communication_handler.process_messages()
                self.data_logger.log_data()
                self.battery_monitor.update()
                self.obstacle_avoidance.check_obstacles()
                self.wind_simulator.update()
                self.sensor_fusion.update()
                self.performance_metrics.update()
                self.simulation_interface.step()
                
                rate.sleep()
            except Exception as e:
                self.error_handler.handle_error("Runtime Error", e)

if __name__ == '__main__':
    simulation = PolliBeeDroneSimulation()
    simulation.run()
