#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


class PController( Node ):

  def __init__( self, kp ):
    super().__init__( 'p_controller' )

    self.kp = kp
    self.saturation = 0.5      
    self.v_lineal = 0.2         
    self.tol_centrado = 25    

    self.setpoint = None
    self.is_goal = False         
    
    self.actuation_pub = self.create_publisher( Twist, '/commands/velocity', 1 )
    self.setpoint_sub = self.create_subscription( Float64, 'setpoint', self.setpoint_cb, 1 )
    self.state_sub = self.create_subscription( Float64, 'state', self.state_cb, 1 )
    self.goal_sub = self.create_subscription( Bool, 'goal_reached', self.goal_cb, 1 )

  def goal_cb( self, msg ):
    self.is_goal = msg.data

  def setpoint_cb( self, msg ):
    self.setpoint = msg.data

  def state_cb( self, msg ):
    if self.setpoint is None:
      return

    state = msg.data
    error = self.setpoint - state

    actuation = self.kp * error

    actuation = max( -self.saturation, min( self.saturation, actuation ) )


    cmd = Twist()

    if not self.is_goal:
      cmd.angular.z = actuation

      # Avanza solo cuando el cuadrado esta casi centrado
      if abs( error ) < self.tol_centrado:
        cmd.linear.x = self.v_lineal

    self.actuation_pub.publish( cmd )
    self.get_logger().info( f'Error: {error:.2f} | Actuacion (w): {actuation:.2f} rad/s' )


def main():
  rclpy.init()
  p_ctrl = PController( kp=0.03 )
  rclpy.spin( p_ctrl )


if __name__ == '__main__':
  main()
