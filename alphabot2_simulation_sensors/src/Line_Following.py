#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

pub_ = None

def callback(sensor1, sensor2, sensor3, sensor4, sensor5):
    global pub_;

    rangesSensor1 = sensor1.ranges.size()
    rangesSensor2 = sensor2.ranges.size()
    rangesSensor3 = sensor3.ranges.size()
    rangesSensor4 = sensor4.ranges.size()
    rangesSensor5 = sensor5.ranges.size()

    msg = Twist()

    if(sensor1.ranges[rangesSensor1 - 1] == math.inf || sensor2.ranges[rangesSensor2 - 1] == math.inf ||
        sensor3.ranges[rangesSensor3 - 1] == math.inf || sensor4.ranges[rangesSensor4 - 1] == math.inf ||
        sensor5.ranges[rangesSensor5 - 1] == math.inf):
        # Walk to right or left to search for line
    else:
        msg.linear.x = 0.35
        msg.angular.z = 0.0

    pub_.publish(msg);

def main():

    global pub_;

    rospy.init_node('alphabot2_simulation')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub1 = message_filters.Subscriber('/alphabot2/laser/scan/sensor1', LaserScan)
    sub2 = message_filters.Subscriber('/alphabot2/laser/scan/sensor2', LaserScan)
    sub3 = message_filters.Subscriber('/alphabot2/laser/scan/sensor3', LaserScan)
    sub4 = message_filters.Subscriber('/alphabot2/laser/scan/sensor4', LaserScan)
    sub5 = message_filters.Subscriber('/alphabot2/laser/scan/sensor5', LaserScan)

    ts = message_filters.TimeSynchronizer([sub1, sub2, sub3, sub4, sub5], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    main()
