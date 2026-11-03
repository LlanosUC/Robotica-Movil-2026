#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from tf_transformations import euler_from_quaternion
import math

class StateCtrl(Node):

    def __init__(self):
        super().__init__('state_ctrl')

        self.state_pub = self.create_publisher(
            Float64,
            'state',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_cb, 
            10)

        self.ctrl_sub = self.create_subscription(
           Bool,
           'ctrl',
           self.reset_cb,
           1
        )

        self.state = Float64()
        self.pose_inicial = (0.0, 0.0, 0.0)
        self.pose_actual = (0.0, 0.0, 0.0)
        self.disp = True # True : lineal, False : rotacional

        
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        _, _, yaw = euler_from_quaternion( ( msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w ) )

        self.pose_actual = (x, y, yaw)



        dx = self.pose_actual[0] - self.pose_inicial[0]
        dy = self.pose_actual[1] - self.pose_inicial[1]

        d = math.hypot(dx, dy) ## distancia euclidiana

        dtheta = self.pose_actual[2] - self.pose_inicial[2]

        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))


        if self.disp:
            self.state.data = float(d)
        else:
            self.state.data = float(dtheta)

        self.state_pub.publish(self.state)

    def reset_cb(self, msg):

       self.pose_inicial = self.pose_actual
       self.disp = msg.data

       
def main( args = None ):
  rclpy.init( args = args )

  state_ctrl = StateCtrl()

  rclpy.spin( state_ctrl )
  state_ctrl.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

