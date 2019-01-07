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
HIGH_LINEAR_SPEED = 0.8
LOW_LINEAR_SPEED = 0.6

HIGH_ANGULAR_SPEED = 2.1
LOW_ANGULAR_SPEED = 1.5


movementTopic = None
bottomSensors = None
topSensors = None
lock = Lock()

lineNeverFound = True
prev_speed = 0
prev_z_sign = 0

def sign(x):
    if x < 0: return -1
    elif x > 0: return 1
    else: return 0

def calculateMovement(sensorsTop, sensorsBottom):
    global movementTopic
    global lineNeverFound
    global prev_speed
    global prev_z_sign

    msg = Twist()
    obstacleRight, obstacleLeft = sensorsTop.data
    print(sensorsTop.data)
    if obstacleLeft: #TODO should this also consider obstacleRight and obstacleLeft simultaneously?
        print "Obstacle found on the left. Turning right"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = - HIGH_ANGULAR_SPEED
    elif obstacleRight:
        print "Obstacle found on the right. Turning left"
        msg.linear.x = LOW_LINEAR_SPEED
        msg.angular.z = HIGH_ANGULAR_SPEED
    else:
        # print "Everything fine"

        brightness1, brightness2, brightness3, brightness4, brightness5 = sensorsBottom.data
        print(sensorsBottom.data)
        #'''
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
            #if brightness3 > 50:
            lineNeverFound = False
            if abs(mid_deviation) < 50 and max(brightness2, brightness4) > 50:
                print("Staying in the line")
                msg.linear.x = LOW_LINEAR_SPEED
                msg.angular.z = 0

            elif abs(edge_deviation) < 50 and max_brightness > 50: # little hope
                print("Warning! Correcting course to the "+str_turn)
                msg.linear.x = LOW_LINEAR_SPEED
                msg.angular.z = z_sign*HIGH_ANGULAR_SPEED
            else: # true panic
                print("PANICKING while correcting course!")
                z_sign = prev_z_sign
                msg.linear.x = - LOW_LINEAR_SPEED
                msg.angular.z = z_sign*LOW_ANGULAR_SPEED
                #msg.angular.z = z_sign*LOW_ANGULAR_SPEED
                #msg.linear.x = HIGH_LINEAR_SPEED*1.4
                #msg.angular.z = LOW_ANGULAR_SPEED
        #'''
        #msg.angular.z = LOW_ANGULAR_SPEED
        prev_z_sign = z_sign
    s1 = sign(prev_speed)
    s2 = sign(msg.linear.x)
    if s1*s2 < 0:
        msg.linear.x = prev_speed + msg.linear.x
    else:
        msg.linear.x = (prev_speed+msg.linear.x)/3
    prev_speed = msg.linear.x
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

    movementTopic = rospy.Publisher('/alphabot2/control', Twist, queue_size=1)

    rospy.Subscriber('/alphabot2/top_sensors', Int32MultiArray, callbackTop)
    rospy.Subscriber('/alphabot2/bottom_sensors', Int32MultiArray, callbackBottom)

    #sub1 = message_filters.Subscriber('/alphabot2/top_sensors', Int32MultiArray)
    #sub2 = message_filters.Subscriber('/alphabot2/bottom_sensors', Int32MultiArray)
    #ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    #ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
