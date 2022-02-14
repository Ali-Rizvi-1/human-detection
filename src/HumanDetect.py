#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.puber = rospy.Publisher('centeroid', Float64MultiArray)
    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
    except CvBridgeError as e:
      print(e)

    newimage = cv_image

    (rows,cols,channels) = cv_image.shape

    # human detection 
    # initialize the HOG descriptor/person detector
    HOGCV = cv2.HOGDescriptor()
    HOGCV.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    
    # detectByPathImage(path)

    bounding_box_cordinates, weights =  HOGCV.detectMultiScale(cv_image, winStride = (4, 4), padding = (8, 8), scale = 1.03)
    # msg_list = int_array2d()
    data_to_send = Float64MultiArray()
    centeroid = []
    person = 1
    for x,y,w,h in bounding_box_cordinates:
        cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,255,0), 2)
        # cv2.putText(cv_image, f'person {person}', (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        centeroid_x = x+w/2
        centeroid_y = y+h/2
        # cv2.circle(frame,(centeroid_x, centeroid_y), 25, (0,255,0))
        centeroid.append(centeroid_x) #[centeroid_x,centeroid_y]
        centeroid.append(centeroid_y)
        person += 1
    
    data_to_send.data = centeroid
    cv2.putText(cv_image, 'Status : Detecting ', (40,40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255,0,0), 2)
    # cv2.putText(cv_image, f'Total Persons : {person-1}', (40,70), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255,0,0), 2)
    cv2.imshow("Image window",cv_image)

    cv2.waitKey(3)
    # cv2.destroyAllWindows()

    # print(centeroid)
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    # self.puber.publish(centeroid)

    try:
      self.puber.publish(data_to_send)
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(newimage, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter_detection', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
