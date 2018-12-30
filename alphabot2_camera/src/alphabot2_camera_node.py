#!/usr/bin/env python

import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class camera_node:

    def __init__(self):
        rospy.init_node('camera_node')

        """ Give the OpenCV display window a name. """
        self.cv_window_name = "OpenCV Image"

        """ Create the window and make it re-sizeable (second parameter = 0) """
        cv.namedWindow(self.cv_window_name, 0)

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber("alphabot2_camera/image_raw", Image, self.callback)

            

    def callback(self, data):
        try:
            """ Convert the raw image to OpenCV format """
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
          print e
  
        
        """ Get the width and height of the image """
        (width, height) = cv.GetSize(cv_image)

        """ Overlay some text onto the image display """
        text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 2, 2)
        cv.PutText(cv_image, "OpenCV Image", (50, height / 2), text_font, cv.RGB(255, 255, 0))
  
        """ Refresh the image on the screen """
        cv.ShowImage(self.cv_window_name, cv_image)
        cv.WaitKey(3)
    
def main(args):
      vn = camera_node()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down vison node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
