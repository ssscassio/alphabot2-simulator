#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import time
import RPi.GPIO as GPIO

message_to_send_to_simulation = Twist()
message_to_send_to_alphabot = Twist()

IN1 = 13
IN2 = 12
IN3 = 21
IN4 = 40
ENA = 6
ENB = 26
PA  = 50
PB  = 50

class driver:
  def __init__(self):
    self.IN1 = 13
    self.IN2 = 12
    self.IN3 = 21
    self.IN4 = 40
    self.ENA = 6
    self.ENB = 26
    self.PA  = 50
    self.PB  = 50

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(self.IN1, GPIO.OUT)
    GPIO.setup(self.IN2, GPIO.OUT)
    GPIO.setup(self.IN3, GPIO.OUT)
    GPIO.setup(self.IN4, GPIO.OUT)
    GPIO.setup(self.ENA, GPIO.OUT)
    GPIO.setup(self.ENB, GPIO.OUT)
    self.PWMA = GPIO.PWM(self.ENA,500)
    self.PWMB = GPIO.PWM(self.ENB,500)
    self.PWMA.start(self.PA)
    self.PWMB.start(self.PB)
    self.stop()

  def stop(self):
    self.PWMA.ChangeDutyCycle(0)
    self.PWMB.ChangeDutyCycle(0)
    GPIO.output(self.IN1, GPIO.LOW)
    GPIO.output(self.IN2, GPIO.LOW)
    GPIO.output(self.IN3, GPIO.LOW)
    GPIO.output(self.IN4, GPIO.LOW)


  def set_motor(self, left, right):
    print('Set motor left={} right={}'.format(left, right))

    if (right >= 0) and (right <= 100):
      GPIO.output(self.IN1, GPIO.HIGH)
      GPIO.output(self.IN2, GPIO.LOW)
      self.PWMA.ChangeDutyCycle(right)
    elif (right < 0) and (right >= -100):
      GPIO.output(self.IN1, GPIO.LOW)
      GPIO.output(self.IN2, GPIO.HIGH)
      self.PWMA.ChangeDutyCycle(0 - right)
    if (left >= 0) and (left <= 100):
      GPIO.output(self.IN3, GPIO.HIGH)
      GPIO.output(self.IN4, GPIO.LOW)
      self.PWMB.ChangeDutyCycle(left)
    elif (left < 0) and (left >= -100):
      GPIO.output(self.IN3, GPIO.LOW)
      GPIO.output(self.IN4, GPIO.HIGH)
      self.PWMB.ChangeDutyCycle(0 - left)

  # get cmd_vel message, and get linear velocity and angular velocity
  def drive(self, data):
    x = data.linear.x
    angular = data.angular.z
    # calculate right and left wheels' signal
    right = int((x + angular) * 50)
    left = int((x - angular) * 50)
    self.set_motor(left, right)



def callback(alphabot2_cmd_vel):
  global message_to_send_to_alphabot, message_to_send_to_simulation
  # Sending to Simulator
  message_to_send_to_simulation = alphabot2_cmd_vel
  gazebo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  gazebo_pub.publish(message_to_send_to_simulation)
  
  # Sending to Alphabot2
  d = driver()
  d.drive(data)
  
  

def main():
  rospy.init_node('alphabot_control', anonymous= True)

  sub = rospy.Subscriber('/alphabot2_cmd_vel', Twist, callback)
  rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

