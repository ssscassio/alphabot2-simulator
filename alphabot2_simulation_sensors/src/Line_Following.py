#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import random

pub_ = None
last_vel = [random.uniform(0.1,0.3),  random.uniform(-0.3,0.3)]

def callback(sensor1, sensor2, sensor3, sensor4, sensor5):
    global pub_, last_vel;

    # ver se ranges é o mesmo que samples
    rangesSensor1 = sensor1.ranges.size()
    rangesSensor2 = sensor2.ranges.size()
    rangesSensor3 = sensor3.ranges.size()
    rangesSensor4 = sensor4.ranges.size()
    rangesSensor5 = sensor5.ranges.size()

    print "Ranges size sensor 1: "
    print rangesSensor1

    print "Ranges size sensor 2: "
    print rangesSensor2

    print "Ranges size sensor 3: "
    print rangesSensor3

    print "Ranges size sensor 4: "
    print rangesSensor4

    print "Ranges size sensor 5: "
    print rangesSensor5


    print "Sensor 1 value: "
    print sensor1.ranges[rangesSensor1 - 1]

    print "Sensor 2 value: "
    print sensor2.ranges[rangesSensor2 - 1]

    print "Sensor 3 value: "
    print sensor3.ranges[rangesSensor3 - 1]

    print "Sensor 4 value: "
    print sensor4.ranges[rangesSensor4 - 1]

    print "Sensor 5 value: "
    print sensor5.ranges[rangesSensor5 - 1]
    

    msg = Twist()

    if (sensor1.ranges[rangesSensor1 - 1] == math.inf || sensor2.ranges[rangesSensor2 - 1] == math.inf ||
        sensor3.ranges[rangesSensor3 - 1] == math.inf || sensor4.ranges[rangesSensor4 - 1] == math.inf ||
        sensor5.ranges[rangesSensor5 - 1] == math.inf):

        print
        # random wandering
         msg.linear.x = max(min( last_vel[0] + random.uniform(-0.01,0.01),0.3),0.1)
         msg.angular.z= max(min( last_vel[1] + random.uniform(-0.1,0.1),1),-1)
         if msg.angular.z == 1 or msg.angular.z == -1:
             msg.angular.z = 0
        last_vel[0] = msg.linear.x
        last_vel[1] = msg.angular.z
    else:
        # line following
        msg.linear.x = 0.35
        msg.angular.z = 0.0 # calculate right value

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
