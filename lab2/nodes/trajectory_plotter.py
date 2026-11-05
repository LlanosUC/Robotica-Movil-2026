#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import csv
import os
import numpy as np

def mse_compute(x_ref, y_ref, x_real, y_real):

    if len(x_ref) == 0 or len(x_real) == 0:
        return 0.0

    ref_points = np.column_stack((x_ref, y_ref)) 
    real_points = np.column_stack((x_real, y_real))

    diff = real_points[:, np.newaxis, :] - ref_points[np.newaxis, :, :]

    squared_dist = np.sum(diff**2, axis=2)

    min_squared_error = np.min(squared_dist, axis=1)

    mse = np.mean(min_squared_error)
    return mse

class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__('trajectory_plotter')

        package_share = get_package_share_directory('lab2')
        output_dir = os.path.join(package_share, 'temp_data')

        if not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        self.csv_path = os.path.join(output_dir, 'trayectoria_robot.csv')

        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'origen', 'x', 'y'])
        self.get_logger().info("Logger de trayectorias iniciado...")

        self.path_sub = self.create_subscription(Path, 'nav_plan', self.path_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.goal_sub = self.create_subscription(Bool, 'goal_reached', self.goal_cb, 1)

        self.x_ref, self.y_ref = [], []
        self.x_real, self.y_real = [], []

        
        plt.ion() #para modo interactivo
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_title("Seguimiento de Trayectoria")
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.grid(True)
        self.ax.axis('equal')

        self.line_ref, = self.ax.plot([], [], 'b--', label='Trayectoria Referencia', linewidth=1.5)
        self.line_real, = self.ax.plot([], [], 'r-', label='Trayectoria Real', linewidth=2.0)
        self.ax.legend()

        self.timer = self.create_timer(0.1, self.update_plot)

        self.printed = False

    def path_cb(self, msg):

        t = self.get_clock().now().nanoseconds / 1e9

        self.x_ref = [p.pose.position.x for p in msg.poses]
        self.y_ref = [p.pose.position.y for p in msg.poses]

        for p in msg.poses:
            self.writer.writerow([t, 'path', p.pose.position.x, p.pose.position.y])

        self.line_ref.set_data(self.x_ref, self.y_ref)
        
        if self.x_ref and self.y_ref:
            self.ax.set_xlim(min(self.x_ref) - 2, max(self.x_ref) + 2)
            self.ax.set_ylim(min(self.y_ref) - 2, max(self.y_ref) + 2)

    def goal_cb(self, msg):

        if msg.data and not self.printed:
            mse = mse_compute(self.x_ref, self.y_ref, self.x_real, self.y_real)
            self.get_logger().info(f"MSE: {mse:.5f} m^2")
            self.printed = True

    def odom_cb(self, msg):

        t = self.get_clock().now().nanoseconds / 1e9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.writer.writerow([t, 'odom', x, y])

        self.x_real.append(x)
        self.y_real.append(y)

    def update_plot(self):
        if self.x_real and self.y_real:
            self.line_real.set_data(self.x_real, self.y_real)
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    plotter = TrajectoryPlotter()
    
    rclpy.spin(plotter)

    plt.close('all')
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()