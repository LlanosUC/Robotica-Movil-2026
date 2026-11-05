#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Bool
import csv
import os
from ament_index_python.packages import get_package_share_directory

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        self.declare_parameter('controller', 'P')

        package_share = get_package_share_directory('lab2')
        output_dir = os.path.join(package_share, 'temp_data')

        if not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
        
        self.csv_path = os.path.join(output_dir, 'trayectoria_robot.csv')

        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'origen', 'x', 'y'])

        self.signals_csv_path = os.path.join(output_dir, 'senales_control.csv')

        self.signals_file = open(self.signals_csv_path, 'w', newline='')
        self.signals_writer = csv.writer(self.signals_file)
        self.signals_writer.writerow(['timestamp', 'controlador', 'modo', 'pref', 'vs', 'preal'])

        self.controller = self.get_parameter('controller').value

        self.modo = None
        self.pref = None
        self.preal = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Bool, 'ctrl', self.ctrl_cb, 10)
        self.create_subscription(Float64, 'setpoint', self.setpoint_cb, 10)
        self.create_subscription(Float64, 'state', self.state_cb, 10)
        self.create_subscription(Float64, 'control_effort', self.control_effort_cb, 10)

        self.get_logger().info("Logger de trayectorias y señales de control iniciado...")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow([t, 'odom', x, y])

    def ctrl_cb(self, msg):
        self.modo = 'lineal' if msg.data else 'angular'

    def setpoint_cb(self, msg):
        self.pref = msg.data

    def state_cb(self, msg):
        self.preal = msg.data

    def control_effort_cb(self, msg):
        if self.pref is None or self.preal is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        self.signals_writer.writerow([t, self.controller, self.modo, self.pref, msg.data, self.preal])

    def destroy_node(self):
        self.file.close()
        self.signals_file.close()
        super().destroy_node()


def main( args = None ):
    rclpy.init( args = args )

    pose_logger = PoseLogger()

    rclpy.spin( pose_logger )
    pose_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()