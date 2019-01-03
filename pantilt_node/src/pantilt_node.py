#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import math
import RPi.GPIO as GPIO
import time

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

#Subscribe to:
lower_angle = std_msgs/Float64 #pan - angle in degrees
upper_angle = std_msgs/Float64 #tilt - angle in degrees

#Publish to:
int in_min = -90
int in_max = +90
int MIN_PULSE_WIDTH = 650
int MAX_PULSE_WIDTH = 2350


def get_angle():

def degree_to_rad_lower():
    angle_rad = angle_lower*math.pi/180
    
    pub_lower_angle = rospy.Publisher("/alphabot2/joint_lower_camera_position_controller/command", Float64, queue_size=10)
    rospy.init_node('pub_lower_angle', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(pub_lower_angle)
        pub.publish(pub_lower_angle)
        rate.sleep()

def analog_value(angle):
    pulse_wide = (angle - in_min)*(MAX_PULSE_WIDTH-MIN_PULSE_WIDTH)/(in_max-in_min)+MIN_PULSE_WIDTH
    analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096)
    return analog_value

def degree_to_dutyCycle():
    duttyCycle_lower= analog_value(lower_angle)
    duttyCycle_upper= analog_value(upper_angle)
    p.ChangeDutyCycle(duttyCycle_lower)
    p.ChangeDutyCycle(duttyCycle_upper)
    time.sleep(0.5)


if __name__ == '__main__':
    p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
    p.start(2.5) # Initialization
    get_angle() #Subscribe
     try:
         degree_to_rad_lower() #Publish to simulator
         degree_to_rad_upper() #Publish to simulator
         degree_to_dutyCycle() #Publish to alphabot2
     except rospy.ROSInterruptException:
         pass
