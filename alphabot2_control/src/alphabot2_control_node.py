import rospy
from geometry_msgs.msg import Twist

message_to_send = Twist()

def sensorCallback(message_received):
    global message_to_send_to_alphabot, message_to_send_to_simulation
    message_to_send_to_alphabot = message_received
    message_to_send_to_simulation = message_received


def main():
    global message_to_send
    # Recebendo informação do programa de lógica
    sub = rospy.Subscriber('/cmd_vel', Twist, sensorCallback)


    # Enviando informação para o Simulador
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub.publish(message_to_send)

    # Enviando informação para o Alphabot
    # pub = rospy.Publisher('/action', Twist, queue_size=1)
    # pub.publish(message_to_send)