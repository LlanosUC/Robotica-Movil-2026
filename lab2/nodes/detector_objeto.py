#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2 as cv
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
import cv_bridge

class DetectorObjeto(Node):

    def __init__(self):
        super().__init__('detector_objeto')

        self.bs_position = self.create_publisher(
           UInt16,
           'blue_square_position',
           1
        )

        self.rgb_img = self.create_subscription(
           Image,
           '/camera/rgb/image_color',
           self.img_cb,
           1
        )

        self.bridge = cv_bridge.CvBridge()

    def img_cb(self, msg):

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #https://docs.opencv.org/5.0/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html    
        hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)

        lower_blue = np.array([100, 120, 100])
        upper_blue = np.array([125, 255, 255])

        mask = cv.inRange(hsv, lower_blue, upper_blue)

        #https://docs.opencv.org/5.0/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html#1-moments

        contours, _ = cv.findContours(mask, 1, 2)

        if len(contours) > 0:
            
            # contorno más grande
            block_contour = max(contours, key=cv.contourArea)
            
            if cv.contourArea(block_contour) > 100.0:
                M = cv.moments(block_contour)
                
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    self.bs_position.publish(UInt16(data=cX))


def main( args = None ):
  rclpy.init( args = args )

  detector_objeto = DetectorObjeto()

  rclpy.spin( detector_objeto )
  detector_objeto.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()