import rospy
from std_msgs.msg import String
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge
import imgproc

#include <image_transport/image_transport.h>
#include <algorithm>
#include "geometry_msgs/Twist.h"


class alphabot_camera:
    def __init__(self):
        #TODO

class camera_node:
    def __init__(self):

        """ Subscribe to alphabot2 image_raw """
        self.image_sub = rospy.Subscriber("/alphabot2/raw_image", , self.image_callback)
        #TODO: A IMAGE NÃO ESTÁ A SER PUBLICADA COM ESTE NOME NEM NENHUM... QUE TIPO DE IMAGEM É' cv?

        """ Subscribe to real robot image_raw """
        #TODO

        """ Create Publisher image """
        self.image_pub = rospy.Publisher("/alpahbot2/image", , queue_size=10)
        #TODO TIPO DE IMAGEM A SER PUBLICADA

        """ Initialize Alphabot2 raspicam """
        self.real_robot = alphabot_camera()

    def image_callback():
        """ Publish alphabot2_image """
        #TODO

        
def main():
    cn = camera_node()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.RosInterruptException:
        pass