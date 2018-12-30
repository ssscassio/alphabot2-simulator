#!/usr/bin/env python

import rospy
from std_msgs.msg import String

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
        light = lightSensors.AnalogRead()
        proximity = proximitySensors.getStatus()
        #TO DO: edit return statement to publish somewhere
        return light, proximity


# MOVEMENT

robot = AlphaBot2()

def movementCmdCallback(data): # called by another ROS node
    movType = data.data
    print(movType)
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

MOVEMENT_TOPIC = "alphabot_move"
SENSORS_TOPIC = "alphabot_sensors"

def main():
    # setup
    global lightSensors
    global proximitySensors
    lightSensors = TRSensor()
    proximitySensors = IRSensor()
    proximitySensors.setup(robot)

    # make a broadcaster
    sensorsPub = rospy.Publisher(SENSORS_TOPIC, String, queue_size=10)
    # start node
    rospy.init_node('alphabot_handler', anonymous=True)
    # subscribe movement_listener
    rospy.Subscriber(MOVEMENT_TOPIC, String, movementCmdCallback)

    rate = rospy.Rate(10)
    while True: #TO DO: is there a ROS::ok equivalent for phyton?
        light, proximity = getSensorsInfos()
        printLightSensors(light)
        printProximitySensors(proximity)
        sensorsStr = str(light+proximity)
        print(sensorsStr)

        sensorsPub.publish(sensorsStr)
        rospy.spinOnce()
        rate.sleep()

    #while True:
    #   light, proximity = getSensorsInfos()
    #   printLightSensors(light)
    #   printProximitySensors(proximity)
    #   time.sleep(0.2)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
