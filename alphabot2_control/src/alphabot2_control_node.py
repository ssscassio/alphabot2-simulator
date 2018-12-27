#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

message_to_send_to_simulation = Twist()
message_to_send_to_alphabot = Twist()

def sensorCallback(message_received):
    global message_to_send_to_alphabot, message_to_send_to_simulation
    message_to_send_to_alphabot = message_received
    message_to_send_to_simulation = message_received
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    while True:
        pub.publish(message_to_send_to_simulation)
    print message_to_send_to_simulation


def main():
    global message_to_send_to_simulation
    # Recebendo informacao do programa de logica
    rospy.init_node('alphabot_control', anonymous= True)

    sub = rospy.Subscriber('/alphabot2', Twist, sensorCallback)
    rospy.spin()


    # Enviando informacao para o Simulador
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # rospy.init_node('alphabot_control', anonymous= True)
    # pub.publish(message_to_send_to_simulation)

    #rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #    print message_to_send_to_simulation
    #    pub.publish(message_to_send_to_simulation)
    #    rate.sleep()        
    # Enviando informacao para o Alphabot
    # pub = rospy.Publisher('/action', Twist, queue_size=1)
    # pub.publish(message_to_send_to_alphabot)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
