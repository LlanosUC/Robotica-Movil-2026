#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
import matplotlib.pyplot as plt

class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__('trajectory_plotter')

        # Suscripciones
        self.path_sub = self.create_subscription(Path, 'nav_plan', self.path_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Datos acumulados
        self.x_ref, self.y_ref = [], []
        self.x_real, self.y_real = [], []

        # Configuración de Matplotlib en modo interactivo
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_title("Seguimiento de Trayectoria en Tiempo Real")
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.grid(True)
        self.ax.axis('equal') # Mantiene la proporción de aspecto 1:1

        # Líneas de la gráfica
        self.line_ref, = self.ax.plot([], [], 'b--', label='Trayectoria Referencia', linewidth=1.5)
        self.line_real, = self.ax.plot([], [], 'r-', label='Trayectoria Real', linewidth=2.0)
        self.ax.legend()

        # Timer para actualizar la ventana gráfica (10 Hz)
        self.timer = self.create_timer(0.1, self.update_plot)

    def path_cb(self, msg):
        # Almacena la trayectoria teórica una sola vez
        self.x_ref = [p.pose.position.x for p in msg.poses]
        self.y_ref = [p.pose.position.y for p in msg.poses]
        self.line_ref.set_data(self.x_ref, self.y_ref)
        
        # Ajusta los límites de la ventana a la ruta
        if self.x_ref and self.y_ref:
            self.ax.set_xlim(min(self.x_ref) - 1, max(self.x_ref) + 1)
            self.ax.set_ylim(min(self.y_ref) - 1, max(self.y_ref) + 1)

    def odom_cb(self, msg):
        # Acumula la posición real del robot
        self.x_real.append(msg.pose.pose.position.x)
        self.y_real.append(msg.pose.pose.position.y)

    def update_plot(self):
        if self.x_real and self.y_real:
            self.line_real.set_data(self.x_real, self.y_real)
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    plotter = TrajectoryPlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()