#!/usr/bin/env python
import roslib
roslib.load_manifest('logitech_cam')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

zoom_value = 80
class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    
    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)

   
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    (cols,rows) = cv.GetSize(cv_image)
    zcx = cols/2
    zcy = rows/2
    if cols > 60 and rows > 60 :
      cv.Circle(cv_image, (zcx,zcy), 10, 255)
      
    zoom_value = 100
    M = cv.CreateMat(2, 3, cv.CV_32FC1)
    M[0,0] = zoom_value/100.
    M[0,1] = 0
    M[0,2] = -zcx*(zoom_value/100.-1)
    M[1,0] = 0
    M[1,1] = zoom_value/100.
    M[1,2] = -zcy*(zoom_value/100.-1)
    
    cvi2 = cv_image;
    cv.WarpAffine(cv_image,cvi2,M)
    
    
    cv.ShowImage("Image window", cv_image)
    cv.WaitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  zoom_value = 100
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
