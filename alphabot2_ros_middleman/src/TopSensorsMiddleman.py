#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


pub_ = None

def callback(sensor1, sensor2):
    global pub_

    foundLeft = False
    foundRight = False

    for s in sensor1.ranges:
        if not math.isinf(s):
            foundRight = True

    for s in sensor2.ranges:
        if not math.isinf(s):
            foundLeft = True
    
    if foundRight and not foundLeft:
        print "Turn left"
    elif foundRight and foundLeft:
        print "Obstacle in front"
    elif not foundRight and foundLeft:
        print "Turn right"
    else:
        print "Front"

    #msg = Twist()

    #pub_.publish(msg)


def main():
    global pub_

    rospy.init_node('alphabot2_top_sensors_middleman')

    #pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #TODO fix topic

    sub1 = message_filters.Subscriber('/alphabot2/laser/scan/sensor1_top', LaserScan)
    sub2 = message_filters.Subscriber('/alphabot2/laser/scan/sensor2_top', LaserScan)

    ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
