#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node 
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler

class PoseLoader(Node):

    def __init__(self):
        super().__init__('pose_loader')

        self.pose_pub = self.create_publisher(
            PoseArray,
            'goal_list',
            10 )

        self.path_pub = self.create_publisher(
            Path,
            'nav_plan',
            10
        )
        
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.data_dir = os.path.abspath(os.path.join(script_dir, '..', 'data'))


        # Creación de parámetro para poder seleccionar lista de poses cuadrada o lista arbitraria
        self.declare_parameter('file_name', 'pose_list.txt')
        self.file_name = self.get_parameter('file_name').value
        
        self.path = os.path.join(self.data_dir, self.file_name)
        
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_cb)


    def timer_cb(self):

        array_msg = PoseArray()
        array_msg.poses = self.poses_list()

        if self.file_name == 'pose_list.txt':
            self.pose_pub.publish(array_msg)
        else:
            path_msg = Path()

            for pose_i in array_msg.poses:
                pose_stamped = PoseStamped()
                pose_stamped.pose = pose_i
                path_msg.poses.append(pose_stamped)

            self.path_pub.publish(path_msg)

        self.timer.cancel()

    def poses_list(self):

        poses_list = []

        with open(self.path, "r") as file:

            for linea in file.readlines():
                lista = [float(i) for i in linea.strip().split(',')]

                if len(lista) == 2:
                    pose = self.create_pose(lista[0]-1, lista[1]-1, theta=0.0)
                elif len(lista) >=3:
                    pose = self.create_pose(lista[0], lista[1], theta=lista[2])
                else:
                    continue
                poses_list.append(pose)

        return poses_list

    def create_pose(self, x, y, theta=0.0):
        pose = Pose()

        pose.position.x = x
        pose.position.y = y

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

def main( args = None ):
  rclpy.init( args = args )

  pose_loader = PoseLoader()

  rclpy.spin( pose_loader )

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  pose_loader.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()