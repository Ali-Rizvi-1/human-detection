#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import matplotlib.pyplot as plt
import math 
count = [0,0]
# f = #focal length
# ppi = # pixels per inch  
radial = 0.0
theta = 0.0
class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_3",Image)

    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber("/camera/depth_aligned_to_color_and_infra1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)
    
    #####################         CODE STARTS          ##########################

    # R and C recieved from human detection node, add subcriber above
    r = 150
    c = 330
 
    (rows,cols) = cv_image.shape
    #create numpy list of image for simplicity
    lst = []
    for i in range(0,rows-1):
        tlst = []
	for j in range(0,cols-1):
           tlst.append(cv_image[i,j])
        lst.append(tlst)
    lst = np.array(lst)
    # Gray image is only for visualization, for all purposes,
    # use lst to get depth values from image
    uint_img = np.array(lst/255).astype('uint8')
    gray = cv2.cvtColor(uint_img, cv2.COLOR_GRAY2BGR)

    #print(lst[r,c])
    cv2.line(gray,(c,r),(c,r),(255,0,0),10)    # remember when visualizing you give (c,r) and (r,c) when indexing
    cv2.imshow("Image window", gray)
    cv2.waitKey(3)
    time.sleep(0.2)
    z = float(lst[r,c]) #+620.0#/ 1000.0 # in meters   

    #x_slice = lst[r]
    #plt.figure
    #plt.plot(x_slice)
    #plt.show()
    #print(x_slice)
    
    # localization local coor
    # f is focal length and ppi is pixels per inch
    # l_p = pixes/ppi
    # l_p = lp*    # covert to meters
    cx = 320.0
    fx = 343.496
    u=float(c)
    x = z*(c - cx)/fx   
    

    # global coordinates
    #(x,y)

    # visualize - marker array
    # red big dabba at (x,y)


    #print(x,y)
    print("x = ", z/1000.0, "and y = ", x/1000.0)
    #try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter_localization', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
