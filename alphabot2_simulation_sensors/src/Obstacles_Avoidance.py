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

    rangesSensor1 = sensor1.ranges.size()
    rangesSensor2 = sensor2.ranges.size()

    msg = Twist()

    if (sensor1.ranges[0] != math.inf || sensor1.ranges[rangesSensor1/3 - 1] != math.inf):
        # Left Obstacle -> try to walk right
    elif (sensor1.ranges[rangesSensor1/3 * 2] != math.inf || sensor2.ranges[rangesSensor2/3 * 2] != math.inf):
        # Front Obstacle -> try to walk right
    elif (sensor2.ranges[rangesSensor2/3 * 2 + 1] != math.inf || sensor2.ranges[rangesSensor2 - 1] != math.inf):
        # Right Obstacle -> trye to walk left
    else:
        msg.linear.x = 0.35
        msg.angular.z = 0.0 # calculate right value

    pub_.publish(msg);

def main():
    global pub_

    rospy.init_node('alphabot2_simulation')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub1 = message_filters.Subscriber('/alphabot2/laser/scan/sensor1_top', LaserScan)
    sub2 = message_filters.Subscriber('/alphabot2/laser/scan/sensor2_top', LaserScan)

    ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
