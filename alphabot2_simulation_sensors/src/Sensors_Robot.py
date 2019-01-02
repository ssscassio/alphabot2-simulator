#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

pub_ = None

def callbackTop(sensor1, sensor2):
    global pub_

    msg = Twist()

    pub_.publish(msg)

# NOT YET USED
def callbackBottom(sensor1, sensor2, sensor3, sensor4, sensor5):
    global pub_

    msg = Twist()

    pub_.publish(msg)

def main():
    global pub_

    rospy.init_node('alphabot2_simulation')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub1 = message_filters.Subscriber('/alphabot2/laser/scan/sensor1_top', LaserScan)
    sub2 = message_filters.Subscriber('/alphabot2/laser/scan/sensor2_top', LaserScan)

    ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    ts.registerCallback(callbackTop)

    rospy.spin()

if __name__ == '__main__':
    main()
