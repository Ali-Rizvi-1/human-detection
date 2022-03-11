#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import numpy as np
import time
import matplotlib.pyplot as plt
import math
import simplejson
count = [0,0]
# f = #focal length
# ppi = # pixels per inch  
radial = 0.0
theta = 0.0
msg = []
T = np.matrix([[0.0,0.0,0.0],[0.0,0.0,0.0]])
T_coordinate = []
T_coordinate_slam = []
coord = []

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_3",Image)
    #
    self.bridge = CvBridge()

   
    self.image_center = rospy.Subscriber("/centeroid", Float64MultiArray, self.callback2)
    self.image_sub = rospy.Subscriber("/camera/depth_aligned_to_color_and_infra1/image_raw",Image,self.callback)
    #self.drone_pose = rospy.Subscriber("/mavros/global_position/local", Odometry, self.callback3)
    self.drone_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback3)

    self.drone_slam_pose = rospy.Subscriber("/rtabmap/odom", Odometry, self.callback4)

  # def callback4(self, data): # poses from SLAM

  #   pass

  def callback4(self,data):
    q0 = data.pose.pose.orientation.w
    q1 = data.pose.pose.orientation.x
    q2 = data.pose.pose.orientation.y
    q3 = data.pose.pose.orientation.z
    x1 = data.pose.pose.position.y
    y1 = data.pose.pose.position.x
    z1 = data.pose.pose.position.z
    drone_slam_pose = open(r"/home/ali/catkin_ws/src/human-detection/src/drone_slam_pose.txt", "a")
    # drone_slam_pose.write([x1,y1,z1])
    simplejson.dump([x1,y1,z1], drone_slam_pose)
    drone_slam_pose.write("\n")
    drone_slam_pose.close()
    yaw   = np.arctan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));
    print('callback4 coord ',coord)
    if len(coord) != 0:
      for j in range(len(coord)):
        if (j%2.0) == 0.0:
          Tx = coord[j]*math.cos(yaw) + coord[j+1]*math.sin(yaw) + x1
          Ty = coord[j]*math.sin(yaw) - coord[j+1]*math.cos(yaw) + y1
       
          if len(T_coordinate_slam) != 0:
            flag_x = 0
            flag_y = 0
            for i in range(len(T_coordinate_slam)):
              #print(T_coordinate[i])
              if Tx-2.5 < T_coordinate_slam[i][0] < Tx+2.5:
                flag_x = 1
              if Ty-2.5 < T_coordinate_slam[i][1] < Ty+2.5 :
                flag_y = 1
            if flag_x == 0 or flag_y == 0:
              T_coordinate_slam.append([Tx,Ty])
              human_slam_pose = open(r"/home/ali/catkin_ws/src/human-detection/src/human_slam_pose.txt", "a")
              simplejson.dump([Tx,Ty], human_slam_pose)
              human_slam_pose.write("\n")
              # human_slam_pose.write([Tx,Ty])
              human_slam_pose.close()
            #print('flag')
            #print([flag_x, flag_y])
          else:
            T_coordinate_slam.append([Tx,Ty])
            human_slam_pose = open(r"/home/ali/catkin_ws/src/human-detection/src/human_slam_pose.txt", "a")
            simplejson.dump([Tx,Ty], human_slam_pose)
            human_slam_pose.write("\n")
            # human_slam_pose.write([Tx,Ty])
            human_slam_pose.close()

          #print(yaw)
          #print(coord)
    print('global slam',T_coordinate_slam)

  def callback3(self,data):
 
    print('callback3')  
    for i in range(len(data.name)):
      if data.name[i] == 'iris':
        ind = i
        break
    #print(ind)
    print(data.pose[ind])
    q0 = data.pose[ind].orientation.w
    q1 = data.pose[ind].orientation.x
    q2 = data.pose[ind].orientation.y
    q3 = data.pose[ind].orientation.z
    x1 = data.pose[ind].position.x
    y1 = data.pose[ind].position.y
    z1 = data.pose[ind].position.z
    drone_gazebo_pose = open(r"/home/ali/catkin_ws/src/human-detection/src/drone_gazebo_pose.txt", "a")
    # drone_gazebo_pose.write([x1,y1,z1])
    simplejson.dump([x1,y1,z1], drone_gazebo_pose)
    drone_gazebo_pose.write("\n")
    drone_gazebo_pose.close()
    yaw   = np.arctan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));
    #print('callback3 coord ',coord)
    if len(coord) != 0:
      for j in range(len(coord)):
        if (j%2.0) == 0.0:
          Tx = coord[j]*math.cos(yaw) + coord[j+1]*math.sin(yaw) + x1
          Ty = coord[j]*math.sin(yaw) - coord[j+1]*math.cos(yaw) + y1
       
          if len(T_coordinate) != 0:
            flag_x = 0
            flag_y = 0
            for i in range(len(T_coordinate)):
              #print(T_coordinate[i])
              if Tx-2.0 < T_coordinate[i][0] < Tx+2.0:
                flag_x = 1
              if Ty-2.0 < T_coordinate[i][1] < Ty+2.0 :
                flag_y = 1
            if flag_x == 0 or flag_y == 0:
              T_coordinate.append([Tx,Ty])
              human_gazebo_pose = open(r"/home/ali/catkin_ws/src/human-detection/src/human_gazebo_pose.txt", "a")
              # human_gazebo_pose.write([Tx,Ty])
              simplejson.dump([Tx,Ty], human_gazebo_pose)
              human_gazebo_pose.write("\n")
              human_gazebo_pose.close()
            #print('flag')
            #print([flag_x, flag_y])
          else:
            T_coordinate.append([Tx,Ty])
            human_gazebo_pose = open(r"/home/ali/catkin_ws/src/human-detection/src/human_gazebo_pose.txt", "a")
            # human_gazebo_pose.write([Tx,Ty])
            simplejson.dump([Tx,Ty], human_gazebo_pose)
            human_gazebo_pose.write("\n")
            human_gazebo_pose.close()

          #print(yaw)
          #print(coord)
    print('global ',T_coordinate)

   

  def callback2(self,data):
   # centroid coordinates from the human detect node 
    print('callback2')
 
   
    for i in range(len(data.data)/2):
      if (i%2.0) == 0.0:
        if data.data[i] not in msg:
          msg.append(data.data[i])
          msg.append(data.data[i+1])
   
   # T_coordinate = np.dot(T, np.transpose(msg))
 


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)
    # msg = self.image_center.data
    # print(msg)
    #####################         CODE STARTS          ##########################
    # print('callback')
    # print(msg)

    # R and C recieved from human detection node, add subcriber above
    for k in range(len(msg)/2):
     
      if (k%2.0 )== 0.0:
        a = 0.0
        b = 0.0
        r = int(msg[k+1])
        c = int(msg[k])
 
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
       
        # localization local coor
        # f is focal length and ppi is pixels per inch
        # l_p = pixes/ppi
        # l_p = lp*    # covert to meters
        cx = 320.0
        fx = 343.496
        ry = 320.0
        fy = 343.496
        u=float(c)
        x = z*(c - cx)/fx 

        y = z*(r-ry)/fy 
       

        # global coordinates
        #(x,y)

        # visualize - marker array
        # red big dabba at (x,y)
        a = z/1000.0
        b = x/1000.0
        c = y/1000.0
       
        #print(x,y)
       
        print("x = ", z/1000.0, "and y = ", x/1000.0, "and z = ", y/1000.0)
      if a not in coord:
        if a != 0.0 and b != 0.0: # and c != 0.0
          coord.append(a)
          coord.append(b)
          # coord.append(c)
      print('coord ',coord)
      print(msg)
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
