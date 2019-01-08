#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
import time

class pan_tilt_node:
  def __init__(self):
    rospy.init_node('alphabot2_pantilt_node_gazebo', anonymous=True)

    """ Subscribe to alphabot2 vertical and horizontal camera controller (Angles in degrees)"""
    self.vertical_sub = rospy.Subscriber("/alphabot2/vertical", Float64, self.vertical_callback)
    self.horizontal_sub = rospy.Subscriber("/alphabot2/horizontal", Float64, self.horizontal_callback)

    """ Create Gazebo Publishers """
    self.gazebo_vertical_pub = rospy.Publisher("/alphabot2/joint_upper_camera_position_controller/command", Float64, queue_size=10)
    self.gazebo_horizontal_pub = rospy.Publisher("/alphabot2/joint_lower_camera_position_controller/command", Float64, queue_size=10)

  def vertical_callback(self, angle_degree):
    angle_rad = self.convert_degree_to_rad(angle_degree)
    
    """ Publishing to Gazebo """
    self.gazebo_vertical_pub.publish(angle_rad)

  def horizontal_callback(self, angle_degree):
    angle_rad = Float64(self.convert_degree_to_rad(angle_degree))

    """ Publishing to Gazebo """
    self.gazebo_horizontal_pub.publish(angle_rad)

  def convert_degree_to_rad(self, angle_degree):
    return angle_degree.data*math.pi/180

def main():
  ptn = pan_tilt_node()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
