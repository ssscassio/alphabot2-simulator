#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(sensor1, sensor2):

def main():
    rospy.init_node('alphabot2_simulation')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub1 = message_filters.Subscriber('/alphabot2/laser/scan/sensor1_top', LaserScan)
    sub2 = message_filters.Subscriber('/alphabot2/laser/scan/sensor2_top', LaserScan)

    ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    main()
