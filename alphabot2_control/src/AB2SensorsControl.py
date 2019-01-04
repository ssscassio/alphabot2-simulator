#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from threading import Lock

HIGH_LINEAR_SPEED = 1
LOW_LINEAR_SPEED = 0.2

HIGH_ANGULAR_SPEED = 1
LOW_ANGULAR_SPEED = 0.3


movementTopic = None
bottomSensors = None
topSensors = None
lock = Lock()

def calculateMovement(sensorsTop, sensorsBottom):
    global movementTopic

    msg = Twist()
    obstacleRight, obstacleLeft = sensorsTop.data
    if obstacleLeft: #should this also consider obstacleRight and obstacleLeft simultaneously?
        print "Obstacle found on the left. Turning right"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = - HIGH_ANGULAR_SPEED
    elif obstacleRight:
        print "Obstacle found on the right. Turning left"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = HIGH_ANGULAR_SPEED
    else:
        print "Everything fine"
        msg.linear.x = HIGH_LINEAR_SPEED
        msg.angular.z = 0

    #TODO consider bottom sensors

    brightness1, brightness2, brightness3, brightness4, brightness5 = sensorBottom.data

    # checks if the middle sensor scans the line (it's mandatory to happen) and if the other sensors scan also the respective line
    if (brightness3 == 100 && (brightness1 == 100 || brightness2 == 100 || brightness4 == 100 || brightness5 == 100)):
        print "Line following"
        msg.linear.x = HIGH_LINEAR_SPEED
        msg.angular.z = 0
    elif (brightness1 > brightness5): # first sensor (most right sensor) scans more brightness (it's near the line) than the most left sensor
        print "Line on the right. Turning Right"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = - HIGH_ANGULAR_SPEED
    else: # last sensor (most left sensor) scans more brightness (it's near the line) than the most right sensor or scans the same brightness (robot starts wandering always to the left to search the line)
        print "Line on the left. Turning left"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = HIGH_ANGULAR_SPEED

    movementTopic.publish(msg)


def callbackTop(sensors):
    global topSensors
    global bottomSensors

    #print "Received Top"
    lock.acquire()
    if bottomSensors is not None:
        bottom = bottomSensors
        top = sensors
        bottomSensors = None
        topSensors = None
        lock.release()
        calculateMovement(top, bottom)
    else:
        lock.release()
        topSensors = sensors


def callbackBottom(sensors):
    global topSensors
    global bottomSensors

    #print "Received Bottom"
    lock.acquire()
    if topSensors is not None:
        bottom = sensors
        top = topSensors
        bottomSensors = None
        topSensors = None
        lock.release()
        calculateMovement(top, bottom)
    else:
        lock.release()
        bottomSensors = sensors


def main():
    global movementTopic

    rospy.init_node('alphabot2_gazebo_sensors_control')

    movementTopic = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('/alphabot2/top_sensors', Int32MultiArray, callbackTop)
    rospy.Subscriber('/alphabot2/bottom_sensors', Int32MultiArray, callbackBottom)

    #sub1 = message_filters.Subscriber('/alphabot2/top_sensors', Int32MultiArray)
    #sub2 = message_filters.Subscriber('/alphabot2/bottom_sensors', Int32MultiArray)
    #ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    #ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
