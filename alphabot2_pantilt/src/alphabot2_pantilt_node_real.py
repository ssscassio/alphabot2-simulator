#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from utils.PCA9685 import PCA9685
import math
import RPi.GPIO as GPIO
import time

IN_MIN = -90
IN_MAX = +90
MIN_PULSE_WIDTH = 650
MAX_PULSE_WIDTH = 2350
DEFAULT_PULSE_WIDTH = 1500
FREQUENCY = 50

tilt = 11    #GPIO 17 ==> Tilt Servo
pan = 13     #GPIO 27 ==> Pan Servo
class alphabot_pantilt:
  def __init__(self):
    self.pwm = PCA9685(0x40, debug=False) 
    self.pwm.setPWMFreq(50) # PWM with 50Hz

  def get_pulse_width(self, angle_degree):
    global IN_MIN, IN_MAX, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, DEFAULT_PULSE_WIDTH, FREQUENCY

    pulse_wide = (int(angle_degree) - int(IN_MIN))*(int(MAX_PULSE_WIDTH)-int(MIN_PULSE_WIDTH))/(int(IN_MAX)-int(IN_MIN))+int(MIN_PULSE_WIDTH)
    get_pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096)
    print ("pulseWide=")
    print(get_pulse_width)
    return get_pulse_width

  def move (self, channel, angle_degree) :
    self.pwm.setServoPulse(channel, self.get_pulse_width(angle_degree))

class pan_tilt_node:
  def __init__(self):
    rospy.init_node('alphabot2_pantilt_node_real', anonymous=True)

    """ Subscribe to alphabot2 vertical and horizontal camera controller (Angles in degrees)"""
    self.vertical_sub = rospy.Subscriber("/alphabot2/vertical", Float64, self.vertical_callback)
    self.horizontal_sub = rospy.Subscriber("/alphabot2/horizontal", Float64, self.horizontal_callback)
    
    """ Initialize Alphabot2 Pan Tilt control """
    self.real_robot =  alphabot_pantilt()

  def vertical_callback(self, angle_degree):
    angle_rad = self.convert_degree_to_rad(angle_degree)

    """ Publishing to Alphabot2 """
    self.real_robot.move(0, tilt, angle_degree)

  def horizontal_callback(self, angle_degree):
    angle_rad = Float64(self.convert_degree_to_rad(angle_degree))

    """ Publishing to Alphabot2 """
    self.real_robot.move(1, pan, angle_degree)

  def convert_degree_to_rad(self, angle_degree):
    print ("rad=")
    print(angle_degree.data*math.pi/180)
    return angle_degree.data*math.pi/180

def main():
  ptn = pan_tilt_node()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
