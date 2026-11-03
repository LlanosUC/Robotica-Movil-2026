#!/usr/bin/env python3

import rclpy
import numpy as np
import time
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16, Float64, Bool

class SeguidorObjeto(Node):

    def __init__(self):
        super().__init__('seguidor_objeto')
        self.centro_imagen = ()
        self.bs_position = self.create_subscription(
           UInt16,
           'blue_square_position',  self.cb_bs_position, 10
            
        )
        self.pub_ref_control = self.create_publisher(Float64, 'setpoint', 10)

        self.state_pub = self.create_publisher(
            Float64,
            'state',
            1
        )
 
        self.goal_pub = self.create_publisher(
            Bool,
            'goal_reached',
            1
        )


        self.pos = 0.0               
        self.ultima_deteccion = None
        self.timeout = 0.5    

        self.timer = self.create_timer(0.05, self.timer_cb)
 
    def cb_bs_position(self, msg):
        self.pos = msg.data
        self.ultima_deteccion = self.get_clock().now()
 
    def timer_cb(self):
 
        if self.ultima_deteccion is None:
            perdido = True
        else:
            edad = (self.get_clock().now() - self.ultima_deteccion).nanoseconds / 1e9
            perdido = edad > self.timeout
 
        # goal_reached = True detiene al robot en el controlador
        self.goal_pub.publish(Bool(data=perdido))
 
        # Referencia: cuadrado en el centro de la imagen (posicion 0)
        self.pub_ref_control.publish(Float64(data=0.0))
 
        # Salida de la planta: posicion detectada (dispara el calculo del controlador)
        self.state_pub.publish(Float64(data=0.0 if perdido else float(self.pos)))
 
 
def main( args = None ):
  rclpy.init( args = args )
 
  seguidor_objeto = SeguidorObjeto()
 
  rclpy.spin( seguidor_objeto )
  seguidor_objeto.destroy_node()
  rclpy.shutdown()
 
 
if __name__ == '__main__':
  main()
 
