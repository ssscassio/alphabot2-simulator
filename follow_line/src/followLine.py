import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import math
import time
import cv2 
import numpy as np

#PID control
hz = 20                     # Cycle Frequency
target_rho = 2050            # (3280 × 2464 pixels) Line distance to (0,0) - meddle of the screen
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
old_time = pyb.millis()
i_output = 0
output = 0
vlinear = 1

def HoughTransform(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    lines = cv2.HoughLines(edges,1,np.pi/180,200)
    return lines#for rho,theta in lines[0]:

def sensorcallback(data):
    t, r = HoughTransform(img)
    t_error, r_error = line_to_theta_and_rho_error(img)

    new_result = (t_error * THETA_GAIN) + (r_error * RHO_GAIN)
    delta_result = new_result - old_result
    old_result = new_result

    new_time = pyb.millis()
    delta_time = new_time - old_time
    old_time = new_time

    p_output = new_result
    i_output = max(min(i_output + new_result, I_MAX), I_MIN)
    d_output = (delta_result * 1000) / delta_time
    pid_output = (P_GAIN * p_output) + (I_GAIN * i_output) + (D_GAIN * d_output)

    output = 2/180*(90 + max(min(int(pid_output), 90), -90))-1
    
    velocityToSend.angular.x = 0
    velocityToSend.angular.y = 0
    velocityToSend.angular.z = output

    velocityToSend.linear.x = vlinear
    velocityToSend.linear.y = 0
    velocityToSend.linear.z = 0

    """ Publish alphabot2/control """    
    vel_pub = rospy.Publisher('/alphabot2_control', Twist, queue_size=1)
    while True:
        vel_pub.publish(velocityToSend)
    #print_string = "Line Ok - turn %d - line t: %d, r: %d" % (output, line.theta(), line.rho())
    #else:
    #    print_string = "Line Lost - turn %d" % output

def line_to_theta_and_rho_error(img):
    lines = HoughTransform(img)
    t, r = lines[0] 
    return (t, r - (img.width() // 2))

def main():
        """ Subscribe to image """
        image_transport::Subscriber sub = rospy.Subscriber('/alphabot2_camera', alphabot2_image, sensorCallback)
        

        rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
