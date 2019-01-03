#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

message_to_send_to_simulation = Twist()
message_to_send_to_alphabot = Twist()


def sensorCallback(cmd_vel_received):
    global message_to_send_to_alphabot, message_to_send_to_simulation
    message_to_send_to_alphabot = cmd_vel_received
    message_to_send_to_simulation = cmd_vel_received
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    while True:
        pub.publish(message_to_send_to_simulation)
    print message_to_send_to_simulation


def main():
    global message_to_send_to_simulation

    rospy.init_node('alphabot_control', anonymous= True)

    sub = rospy.Subscriber('/alphabot2', Twist, sensorCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
