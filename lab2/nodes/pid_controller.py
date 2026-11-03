#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from std_msgs.msg import Empty


class PIDController( Node ):

  def __init__( self, kp_lin=0.01, ki_lin = 0, kd_lin = 0, kp_ang = 0.01, ki_ang = 0, kd_ang = 0):
    super().__init__( 'p_controller' )

    self.kp_lin = kp_lin
    self.ki_lin = ki_lin
    self.kd_lin = kd_lin

    self.kp_ang = kp_ang
    self.ki_ang = ki_ang
    self.kd_ang = kd_ang

    self.kp = self.kp_lin
    self.ki = self.ki_lin
    self.kd = self.kd_lin

    self.setpoint = None
    self.state = None
    self.proportional_action = 0
    self.saturation = 0.2

    self.integral = 0.0
    self.last_time = self.get_clock().now()

    self.actuation_pub = self.create_publisher( Float64, 'control_effort', 1 )
    self.dist_set_point_sub = self.create_subscription( Float64, 'setpoint', self.setpoint_cb, 1 )
    self.dist_state_sub = self.create_subscription( Float64, 'state', self.state_cb, 1 )
    self.ctrl_sub = self.create_subscription(Bool, 'ctrl', self.ctrl_cb, 1)

  def ctrl_cb(self, msg):
    es_lineal = msg.data
    if es_lineal:
      self.kp = self.kp_lin
      self.ki = self.ki_lin
      self.kd = self.kd_lin
      self.saturation = 0.2
      self.get_logger().info('[PID] Modo LINEAL activado')

    else:
      self.kp = self.kp_ang
      self.ki = self.ki_ang
      self.kd = self.kd_ang
      self.saturation = 1.0
      self.get_logger().info('[PID] Modo ANGULAR activado')


  def setpoint_cb( self, msg ):
    self.get_logger().info( '[PICTRL] new setpoint received: %.2f' % (msg.data) )
    self.reset()
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

    # Proportional
    p_actuation = self.kp*error

    # Integrative (sin anti-windup)
    i_result = self.integral + (error * dt)
    i_actuation = self.ki*i_result

    # Derivative (Implement me!)
    d_actuation = self.kd*0

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
    msg = Float64()
    msg.data = actuation
    self.actuation_pub.publish( msg )

  def reset( self ):
    self.setpoint = None
    self.state = None
    self.integral = 0.0
    self.last_time = self.get_clock().now()


def main():
  rclpy.init()
  p_ctrl = PIDController(kp_lin=1.0, ki_lin=0.08,
                         kp_ang=1.2, ki_ang=0.03)
  rclpy.spin( p_ctrl )

if __name__ == '__main__':
  main()



