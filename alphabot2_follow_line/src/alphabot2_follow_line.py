#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

import math
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#PID control
hz = 20                     # Cycle Frequency
target_rho = 2050            # (3280 x 2464 pixels) Line distance to (0,0) - meddle of the screen
target_theta = 0.40         # Line angle
max_speed = 0.3             # Maximum speed (m/s)

THETA_GAIN = 40.0
RHO_GAIN = -1.0
P_GAIN = 0.7
I_GAIN = 0.0
I_MIN = -0.0
I_MAX = 0.0
D_GAIN = 0.1

old_result = 0
old_time = time.time()
i_output = 0
output = 0
vlinear = 1


def sensorCallback(data):
    try:
         img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
         print(e)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    #cv2.imshow("Follow_line",edges)
    #cv2.waitKey(2)
    height, width, channels = img.shape
    print height, width, channels
    lines = cv2.HoughLines(edges,2,np.pi/180,100)

    i = 0
    for line in lines:
        rho, theta = line[0]
        #a = np.cos(theta)
        #b = np.sin(theta)
        #x0 = a*rho
        #y0 = b*rho
        #x1 = int(x0 + 1000*(-b))
        #y1 = int(y0 + 1000*(a))
        #x2 = int(x0 - 1000*(-b))
        #y2 = int(y0 - 1000*(a))
        #cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)


        
        if i == 0:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

            i = i+1

        elif abs(rho - last_rho) > 10 or abs(theta - last_theta) > 20 :
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
        
        last_rho = rho
        last_theta = theta

    cv2.imshow("Follow_line",img)
    cv2.waitKey(2)


    #t_error, r_error = line_to_theta_and_rho_error(img)

    #new_result = (t_error * THETA_GAIN) + (r_error * RHO_GAIN)
    #delta_result = new_result - old_result
    #old_result = new_result

    #new_time = time.time()
    #delta_time = new_time - old_time
    #old_time = new_time

    #p_output = new_result
    #i_output = max(min(i_output + new_result, I_MAX), I_MIN)
    #d_output = (delta_result * 1000) / delta_time
    #pid_output = (P_GAIN * p_output) + (I_GAIN * i_output) + (D_GAIN * d_output)

    #output = 2/180*(90 + max(min(int(pid_output), 90), -90))-1
    
    #velocityToSend.angular.x = 0
    #velocityToSend.angular.y = 0
    #velocityToSend.angular.z = output

    #velocityToSend.linear.x = vlinear
    #velocityToSend.linear.y = 0
    #velocityToSend.linear.z = 0

    """ Publish alphabot2/control """   
    #vel_pub = rospy.Publisher('/alphabot2/control', Twist, queue_size=1)
    #while True:
    #    vel_pub.publish(velocityToSend)
    #print_string = "Line Ok - turn %d - line t: %d, r: %d" % (output, line.theta(), line.rho())
    #else:
    #    print_string = "Line Lost - turn %d" % output

def line_to_theta_and_rho_error(img):
    lines = HoughTransform(img)
    t, r = lines[0] 
    return (t, r - (img.width() // 2))

def main():
        global bridge
        bridge = CvBridge()

        """ Initialize control node """
        rospy.init_node('follow_line_node', anonymous= True)
 
        """ Subscribe to image """
        sub = rospy.Subscriber('/alphabot2/camera/image_raw', Image, sensorCallback)
        

        rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
