#!/usr/bin/env python

# import rospy
# from std_msgs.msg import Float64
# from utils.PCA9685 import PCA9685
# import math
# import RPi.GPIO as GPIO
# import time

IN_MIN = -90
IN_MAX = +90
MIN_PULSE_WIDTH = 650
MAX_PULSE_WIDTH = 2350
DEFAULT_PULSE_WIDTH = 1500
FREQUENCY = 50


class alphabot_pantilt:
  def __init__(self):
    self.pwm = PCA9685(0x40, debug=False) 
    self.pwm.setPWMFreq(50) # PWM with 50Hz

  def get_pulse_width(angle_degree):
    global IN_MIN, IN_MAX, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, DEFAULT_PULSE_WIDTH, FREQUENCY

    pulse_wide = (angle_degree - IN_MIN)*(MAX_PULSE_WIDTH-MIN_PULSE_WIDTH)/(IN_MAX-IN_MIN)+MIN_PULSE_WIDTH
    get_pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096)
    return get_pulse_width

  def move (self, channel, angle_degree) :
    self.pwm.setServoPulse(channel, self.get_pulse_width(angle_degree))

class pan_tilt_node:
  def __init__(self):

    """ Subscribe to alphabot vertical and horizontal camera controller (Angles in degrees)"""
    self.vertical_sub = rospy.Subscriber("/alphabot2_vertical", Float64, self.vertical_callback)
    self.horizontal_sub = rospy.Subscriber("/alphabot2_horizontal", Float64, self.horizontal_callback)

    """ Create Gazebo Publishers """
    self.gazebo_vertical_pub = rospy.Publisher("/alphabot2/joint_upper_camera_position_controller/command", Float64, queue_size=10)
    self.gazebo_horizontal_pub = rospy.Publisher("/alphabot2/joint_lower_camera_position_controller/command", Float64, queue_size=10)

    self.real_robot =  alphabot_pantilt()

  def vertical_callback(angle_degree):
    angle_rad = self.convert_degree_to_rad(angle_degree)
    
    """ Publishing to Gazebo """
    self.gazebo_vertical_pub.publish(angle_rad)

    """ Publishing to Alphabot2 """
    self.real_robot.move(0, angle_degree)

  def horizontal_callback(angle_degree):
    angle_rad = self.convert_degree_to_rad(angle_degree)
    
    """ Publishing to Gazebo """
    self.gazebo_horizontal_pub.publish(angle_rad)

    """ Publishing to Alphabot2 """
    self.real_robot.move(1, angle_degree)

  def convert_degree_to_rad(angle_degree):
    return angle_degree*math.pi/180


# def degree_to_rad_lower():
    # angle_rad = angle_lower*math.pi/180
    
    # pub_lower_angle = rospy.Publisher("/alphabot2/joint_lower_camera_position_controller/command", Float64, queue_size=10)
    # rospy.init_node('pub_lower_angle', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     rospy.loginfo(pub_lower_angle)
    #     pub.publish(pub_lower_angle)
    #     rate.sleep()


def main():
  ptn = pan_tilt_node()
  rospy.spin()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
