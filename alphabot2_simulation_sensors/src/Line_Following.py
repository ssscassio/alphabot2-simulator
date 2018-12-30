#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(sensor1, sensor2, sensor3, sensor4, sensor5):

def main():
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
