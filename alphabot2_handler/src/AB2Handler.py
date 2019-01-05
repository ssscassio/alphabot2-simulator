#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

import time
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2

from TRSensors import TRSensor
import Infrared_Obstacle_Avoidance as IRSensor

# DEBUG

def printLightSensors(res):
    print("leftmost: %d, midleft: %d, middle: %d, midright: %d, rightmost: %d" % (
        res[0], res[1], res[2], res[3], res[4]))

def printProximitySensors(res):
    right = "OK" if res[0] == 1 else "!!"
    left = "OK" if res[1] == 1 else "!!"
    print("left %s, right: %s" % (left, right))

# SENSORS

lightSensors = None # below board # higher values -> darker lighting
proximitySensors = None # front of board

def getSensorsInfos():
        light = lightSensors.AnalogRead() #TODO use eadCalibrated()?
        proximity = proximitySensors.getStatus()
        #TO DO: edit return statement to publish somewhere
        return light, proximity


# MOVEMENT

robot = AlphaBot2()


def movementCmdCallback(msg): # called by another ROS node
    print(msg)
    movType = "forward"
    if movType == "forward":
        robot.forward()
    elif movType == "stop":
        robot.stop()
    elif movType == "backward":
        robot.backward()
    elif movType == "rotateLeft":
        robot.left()
    elif movType == "rotateRight":
        robot.right()
    # TO DO: handle more data (?)


# MAIN

def main():
    # setup
    global lightSensors
    global proximitySensors
    lightSensors = TRSensor()
    proximitySensors = IRSensor

    #lightSensors.calibrate()
    proximitySensors.setup(robot)

    # make broadcasters
    topSensorsPub = rospy.Publisher("/alphabot2/top_sensors", Int32MultiArray, queue_size=10)
    bottomSensorsPub = rospy.Publisher("/alphabot2/bottom_sensors", Int32MultiArray, queue_size=10)

    # start node
    rospy.init_node('alphabot2_handler', anonymous=True)

    # subscribe movement_listener
    rospy.Subscriber("/cmd_vel", Twist, movementCmdCallback)

    print "starting loop..."

    rate = rospy.Rate(10)
    while True: #TO DO: is there a ROS::ok equivalent for phyton?
        light, proximity = getSensorsInfos()

        topSensorsPub.publish(Int32MultiArray(data=proximity))
        bottomSensorsPub.publish(Int32MultiArray(data=light))
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
