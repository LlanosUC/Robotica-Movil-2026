#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class PIDController( Node ):

  def __init__( self, kp, ki=0.0, kd=0.0):
    super().__init__( 'p_controller' )

    self.kp = kp
    self.ki = ki
    self.kd = kd

    self.setpoint = None
    self.state = None
    self.proportional_action = 0
    self.saturation = 1.0

    self.integral = 0.0
    self.last_time = self.get_clock().now()

    self.last_error = 0.0

    self.is_goal = False

    self.actuation_pub = self.create_publisher( Twist, '/cmd_vel', 1 )
    self.dist_set_point_sub = self.create_subscription( Float64, 'setpoint', self.setpoint_cb, 1 )
    self.dist_state_sub = self.create_subscription( Float64, 'state', self.state_cb, 1 )
    self.goal_sub = self.create_subscription(Bool, 'goal_reached', self.goal_cb, 1)

  def goal_cb(self, msg):
    self.is_goal = msg.data

  def setpoint_cb( self, msg ):
    self.setpoint = msg.data

  def state_cb( self, msg ):
    if self.setpoint == None:
      return
    

    current_time = self.get_clock().now()
    dt = (current_time - self.last_time).nanoseconds / 1e9
    self.last_time = current_time

    if dt <= 0.0 or dt > 0.5:
      return
    
    self.state = msg.data
    error = self.setpoint - self.state

    error = np.arctan2(np.sin(error), np.cos(error))

    # Proportional
    p_actuation = self.kp*error

    # Integrative (sin anti-windup)
    i_result = self.integral + (error * dt)
    i_actuation = self.ki*i_result

    # Derivative
    d_error = (error - self.last_error) / dt
    d_actuation = self.kd*d_error

    self.last_error = error

    # Actuation
    actuation = p_actuation + i_actuation + d_actuation

    if actuation > self.saturation:
      actuation = self.saturation
      if error < 0:
        self.integral = i_result
    elif actuation < -self.saturation:
      actuation = -self.saturation
      if error > 0:
        self.integral = i_result
    else:
      self.integral = i_result


    # Message sending
    msg = Twist()

    if not self.is_goal:
      msg.linear.x = 0.1
      msg.angular.z = actuation
    else:
      msg.linear.x = 0.0 
      msg.angular.z = 0.0

    self.actuation_pub.publish( msg )

    if not self.is_goal:
      self.get_logger().info(f'Error: {error:.2f} rad | Actuacion (w): {actuation:.2f} rad/s')

  def reset( self ):
    self.setpoint = None
    self.state = None
    self.integral = 0.0
    self.last_error = 0.0
    self.last_time = self.get_clock().now()


def main():
  rclpy.init()
  p_ctrl = PIDController(kp=2.0,ki = 0.01, kd = 0.05)
  rclpy.spin( p_ctrl )

if __name__ == '__main__':
  main()



