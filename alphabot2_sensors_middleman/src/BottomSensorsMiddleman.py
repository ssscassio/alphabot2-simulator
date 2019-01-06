#! /usr/bin/env python

# ROS imports
import rospy
import message_filters
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

algorithmTopic = None

def callback(sensor1, sensor2, sensor3, sensor4, sensor5):
    global algorithmTopic

    msg = Int32MultiArray()
    bridge = CvBridge()
    brightness1 = round(np.mean(cv2.cvtColor(bridge.imgmsg_to_cv2(sensor1, "bgr8"),cv2.COLOR_BGR2GRAY).flatten())*100/255)
    print(brightness1)

    brightness2 = round(np.mean(cv2.cvtColor(bridge.imgmsg_to_cv2(sensor2, "bgr8"),cv2.COLOR_BGR2GRAY).flatten())*100/255)
    print(brightness2)

    brightness3 = round(np.mean(cv2.cvtColor(bridge.imgmsg_to_cv2(sensor3, "bgr8"),cv2.COLOR_BGR2GRAY).flatten())*100/255)
    print(brightness3)

    brightness4 = round(np.mean(cv2.cvtColor(bridge.imgmsg_to_cv2(sensor4, "bgr8"),cv2.COLOR_BGR2GRAY).flatten())*100/255)
    print(brightness4)

    brightness5 = round(np.mean(cv2.cvtColor(bridge.imgmsg_to_cv2(sensor5, "bgr8"),cv2.COLOR_BGR2GRAY).flatten())*100/255)
    print(brightness5)

    msg.data = [brightness1, brightness2, brightness3, brightness4, brightness5]

    algorithmTopic.publish(msg)


def main():
    global algorithmTopic

    rospy.init_node('alphabot2_bottom_sensors_middleman')

    algorithmTopic = rospy.Publisher('/alphabot2/bottom_sensors', Int32MultiArray, queue_size=1)

    sub1 = message_filters.Subscriber('/sensor1_bottom/image_raw', Image)
    sub2 = message_filters.Subscriber('/sensor2_bottom/image_raw', Image)
    sub3 = message_filters.Subscriber('/sensor3_bottom/image_raw', Image)
    sub4 = message_filters.Subscriber('/sensor4_bottom/image_raw', Image)
    sub5 = message_filters.Subscriber('/sensor5_bottom/image_raw', Image)

    ts = message_filters.TimeSynchronizer([sub1, sub2, sub3, sub4, sub5], 10)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
