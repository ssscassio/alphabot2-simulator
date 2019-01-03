#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

algorithmTopic = None

def callback(sensor1, sensor2):
    global algorithmTopic

    msg = Int32MultiArray()
    foundLeft = False
    foundRight = False

    for s in sensor1.ranges:
        if not math.isinf(s):
            foundRight = True

    for s in sensor2.ranges:
        if not math.isinf(s):
            foundLeft = True

    msg.data = [foundRight, foundLeft]


    algorithmTopic.publish(msg)


def main():
    global algorithmTopic

    rospy.init_node('alphabot2_top_sensors_middleman')

    algorithmTopic = rospy.Publisher('/alphabot2/top_sensors', Int32MultiArray, queue_size=1)

    sub1 = message_filters.Subscriber('/alphabot2/laser/scan/sensor1_top', LaserScan)
    sub2 = message_filters.Subscriber('/alphabot2/laser/scan/sensor2_top', LaserScan)

    ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
