#!/usr/bin/env python3

import rclpy
import numpy as np
import time
from rclpy.node import Node 
from geometry_msgs.msg import Twist, PoseArray, Vector3
from std_msgs.msg import Float64, Bool
from tf_transformations import euler_from_quaternion
import threading

class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')
        self.cmd_pub = self.create_publisher( 
            Twist, 
            '/cmd_vel', 
            10 )

        self.pose_sub = self.create_subscription(
            PoseArray, 
            'goal_list', 
            self.accion_mover_cb,
            10 )

        self.occup_sub = self.create_subscription(
            Vector3,
            '/occupancy_state',
            self.occupancy_callback,
            10)

        self.ctrl_pub = self.create_publisher(
            Bool,
            'ctrl',
            1
        )

        self.setpoint_pub = self.create_publisher(
            Float64,
            'setpoint',
            10
        )

        self.actuation_sub = self.create_subscription(
            Float64,
            'control_effort',
            self.actuation_cb,
            10
        )

        #sincronizacion para el envio de comando
        self.actuation_event = threading.Event()
        self.rate = self.create_rate(20.0)

        self.pose_actual = (0.0,0.0,0.0)
        self.obs_detectado = False
        self.actuation = 0.0

    def occupancy_callback(self, msg):
        if (msg.x == 1) or (msg.y == 1) or (msg.z == 1):
            self.obs_detectado = True

            if msg.x == 1:
                self.get_logger().info('obstáculo a la izquierda')
            if msg.y == 1:
                self.get_logger().info('obstáculo en el centro')
            if msg.z == 1:
                self.get_logger().info('obstáculo a la derecha')
        else:
            self.obs_detectado = False

    def accion_mover_cb(self, msg):
        thread_mov = threading.Thread(target=self.procesar_list_poses, args=(msg,))
        thread_mov.start()

    def procesar_list_poses(self, msg):
        for pose in msg.poses:

            x = pose.position.x
            y = pose.position.y


            _, _, yaw = euler_from_quaternion(( pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z,
                                                pose.orientation.w ))
            
            self.mover_robot_a_destino((x,y,yaw))

    def mover_robot_a_destino(self, goal_pose):
    
        displacement_list = []

        # Poses objetivo
        x_G, y_G, theta_G = goal_pose

        x_inicial, y_inicial, theta_inicial = self.pose_actual

        # Diferencia de posición en marco global

        dx = x_G - x_inicial
        dy = y_G - y_inicial
        
        # Cálculo lista de comandos

        # Movimiento en x

        if np.abs(dx) > 1e-4:

            if dx > 0:
                theta_x = 0.0
            else:
                theta_x = np.pi

            dtheta = theta_x - theta_inicial

            # Normalización entre pi y -pi
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))

            if np.abs(dtheta) > 1e-4:

                displacement_list.append((0, dtheta))

                
            displacement_list.append((np.abs(dx), 0))

            theta_actual = theta_x
        else:
            theta_actual = theta_inicial

        # Movimiento en y

        if np.abs(dy) > 1e-4:

            if dy > 0:
                theta_y = np.pi/2
            else:
                theta_y = -np.pi/2

            dtheta = theta_y - theta_actual

            # Normalización entre pi y -pi
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))

            if np.abs(dtheta) > 1e-4:

                displacement_list.append((0, dtheta))

            displacement_list.append((np.abs(dy), 0))
            theta_actual = theta_y

        dtheta = theta_G - theta_actual

        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))

        if abs(dtheta) > 1e-4:
            displacement_list.append((0, dtheta))

        self.aplicar_velocidad(displacement_list)
        self.pose_actual = goal_pose


    def aplicar_velocidad(self, displacement_list):
        cmd_stop = Twist()

        for disp in displacement_list:

            self.actuation_event.clear()
            # Movimiento lineal

            es_lineal = (disp[1] == 0) 
            self.ctrl_pub.publish(Bool(data=bool(es_lineal)))

            setpoint = float(disp[0] if es_lineal else disp[1])
            self.setpoint_pub.publish(Float64(data=setpoint))

            self.actuation_event.wait(timeout=1.0)

            umbral = 0.015 if es_lineal else 0.005

            while np.abs(self.actuation) > umbral:

                speed = Twist()

                if es_lineal:
                    speed.linear.x = self.actuation
                else:
                    speed.angular.z = self.actuation
                
                self.cmd_pub.publish(speed)

                self.rate.sleep()

            self.cmd_pub.publish(cmd_stop)
    
    def actuation_cb(self, msg):

        self.actuation = msg.data
        self.actuation_event.set()


def main( args = None ):
  rclpy.init( args = args )

  dead_reckoning_nav = DeadReckoningNav()

  rclpy.spin( dead_reckoning_nav )
  dead_reckoning_nav.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

