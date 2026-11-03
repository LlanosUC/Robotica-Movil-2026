#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node 
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Bool
from tf_transformations import euler_from_quaternion

class FollowTheCarrot(Node):

    def __init__(self):
        super().__init__('follor_the_carrot')

        self.path_sub = self.create_subscription(
           Path,
           'nav_plan',
           self.path_cb,
           1
        )

        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_cb, 
            10)     

        self.setpoint_pub = self.create_publisher(
           Float64,
           'setpoint',
           1
        )

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

        self.path = None

    def path_cb(self, msg):
        self.path = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def odom_cb(self, msg):

        if self.path is None or len(self.path) == 0:
           return
        
        x_r = msg.pose.pose.position.x
        y_r = msg.pose.pose.position.y

        _, _, yaw = euler_from_quaternion( ( msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w ) )

        robot_pos = np.array([x_r,y_r])

        #indice del punto más cercano al robot
        idx = np.sum((self.path - robot_pos) **2, axis=1).argmin()

        closest_point = self.path[idx]

        lookahead_distance = 0.15

        path_ahead = self.path[idx:]

        if len(path_ahead) > 1:

           diff_vector = np.diff(path_ahead, axis=0)
           distance = np.sqrt(np.sum(diff_vector**2, axis=1))
           acumm_distance = np.cumsum(distance)

           #busca primer indice que supera los 0.3m
           idx_relative = np.searchsorted(acumm_distance, lookahead_distance)

           idx_obj = min(idx + idx_relative + 1, len(self.path) - 1)

           obj_point = self.path[idx_obj]

        else:
           idx_obj = idx
           obj_point = self.path[idx_obj]


        final_point = self.path[-1]
        dist_to_final = np.linalg.norm(final_point - robot_pos)
        goal_tolerance = 0.05

        is_goal = bool(dist_to_final < goal_tolerance)
        self.goal_pub.publish(Bool(data=is_goal))

        if dist_to_final < goal_tolerance:
           theta_ref = yaw
        else:
           x_c, y_c = obj_point[0], obj_point[1]
           theta_ref = np.arctan2(y_c - y_r, x_c - x_r)

        self.setpoint_pub.publish(Float64(data=theta_ref))
        self.state_pub.publish(Float64(data=yaw))
            
def main( args = None ):
  rclpy.init( args = args )

  follow_the_carrot = FollowTheCarrot()

  rclpy.spin( follow_the_carrot )
  follow_the_carrot.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()