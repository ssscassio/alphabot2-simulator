#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from threading import Lock
'''
HIGH_LINEAR_SPEED = 0.7
LOW_LINEAR_SPEED = 0.2

HIGH_ANGULAR_SPEED = 0.9
LOW_ANGULAR_SPEED = 0.3
'''
HIGH_LINEAR_SPEED = 0.4
LOW_LINEAR_SPEED = 0.1

HIGH_ANGULAR_SPEED = 0.6
LOW_ANGULAR_SPEED = 0.1


movementTopic = None
bottomSensors = None
topSensors = None
lock = Lock()

lineNeverFound = True

def sign(x):
    if x < 0: return -1
    elif x > 0: return 1
    else: return 0

def calculateMovement(sensorsTop, sensorsBottom):
    global movementTopic
    global lineNeverFound

    msg = Twist()
    obstacleRight, obstacleLeft = sensorsTop.data
    if obstacleLeft: #TODO should this also consider obstacleRight and obstacleLeft simultaneously?
        print "Obstacle found on the left. Turning right"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = - HIGH_ANGULAR_SPEED
        return
    elif obstacleRight:
        print "Obstacle found on the right. Turning left"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = HIGH_ANGULAR_SPEED
        return
    else:
        # print "Everything fine"

        brightness1, brightness2, brightness3, brightness4, brightness5 = sensorsBottom.data
        print(sensorsBottom.data)

        min_brightness = min(brightness1, brightness2, brightness3, brightness4, brightness5)
        max_brightness = max(brightness1, brightness2, brightness3, brightness4, brightness5)
        mid_deviation = brightness2 - brightness4  # >0 means more white on the right side. ranges from aprox [-100, 100]
        edge_deviation = brightness1 - brightness5 # >0 means more white on the right side
        str_turn = "right" if edge_deviation > 0 else "left"
        z_sign = -sign(edge_deviation)
        print("max: "+str(max_brightness)+", min: "+str(min_brightness)+", mid_dev: "+str(mid_deviation)+", edge_dev: "+str(edge_deviation))
        
        if lineNeverFound and max_brightness < 50:
            print("Looking for the line")
            msg.linear.x = HIGH_LINEAR_SPEED
            msg.angular.z = -0.1
        else:
            lineNeverFound = False
            if min_brightness > 50 and abs(edge_deviation) < 50:
                print("Staying in the line")
                msg.linear.x = HIGH_LINEAR_SPEED
                msg.angular.z = 0

            elif brightness3 > 50 and abs(edge_deviation) < 50: # needs small adjustement
                print("Slightly correcting course to the "+str_turn)
                msg.linear.x = HIGH_LINEAR_SPEED
                msg.angular.z = z_sign*LOW_ANGULAR_SPEED

            elif brightness3 > 50 and abs(mid_deviation) < 50: # needs adjustement, but no panic
                print("Really correcting course to the "+str_turn)
                msg.linear.x = LOW_LINEAR_SPEED
                msg.angular.z = z_sign*HIGH_ANGULAR_SPEED

            elif brightness3 > 50: # almost panic
                print("Hugely! correcting course to the "+str_turn)
                msg.linear.x = 0
                msg.angular.z = z_sign*HIGH_ANGULAR_SPEED
            else: # true panic
                print("PANICKING while correcting course to the "+str_turn)
                msg.linear.x = -LOW_LINEAR_SPEED
                msg.angular.z = z_sign*HIGH_ANGULAR_SPEED

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

    rospy.init_node('alphabot2_sensors_control')

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
