import ctypes
import math
import random
import os
import cv2
import numpy as np

import sys
sys.path.append("/home/fyp/darknet")
import darknet as dn 
import darknet_images as dni 

sys.path.append("/home/fyp/function")
import hsv_fct, yolo_fct
import adv_cone_connect as acc 

net, class_names, class_colors = dn.load_network("/home/fyp/gazebo_ws/src/robot_vision/src/yolov4_cone/yolov4_cones.cfg","/home/fyp/gazebo_ws/src/robot_vision/src/yolov4_cone/cones.data","/home/fyp/gazebo_ws/src/robot_vision/src/yolov4_cone/yolov4_cones.weights",batch_size=1)
# class_names = ["Yellow_Cone","Red_Cone"]
network_width = dn.network_width(net)
network_height = dn.network_height(net)
yolo_rp = ([network_width/2, network_height/2])


def nothing(x):
    pass

cv2.namedWindow("Yellow Cone")
cv2.createTrackbar("L-H", "Yellow Cone", 4, 180, nothing)
cv2.createTrackbar("L-S", "Yellow Cone", 187, 255, nothing)
cv2.createTrackbar("L-V", "Yellow Cone", 160, 255, nothing)
cv2.createTrackbar("U-H", "Yellow Cone", 78, 180, nothing)
cv2.createTrackbar("U-S", "Yellow Cone", 255, 255, nothing)
cv2.createTrackbar("U-V", "Yellow Cone", 255, 255, nothing)

cv2.namedWindow("Red Cone")
cv2.createTrackbar("L-H", "Red Cone", 0, 180, nothing)
cv2.createTrackbar("L-S", "Red Cone", 197, 255, nothing)
cv2.createTrackbar("L-V", "Red Cone", 130, 255, nothing)
cv2.createTrackbar("U-H", "Red Cone", 180, 180, nothing)
cv2.createTrackbar("U-S", "Red Cone", 255, 255, nothing)
cv2.createTrackbar("U-V", "Red Cone", 255, 255, nothing)

#read image and depthmap
path_img = "/home/fyp/Vincent/darknet/data/img/72.jpg"
path_dep = "/home/fyp/Vincent/darknet/data/depthmap/d72.png"
image = cv2.imread(path_img)
depthimg = cv2.imread(path_dep)
# image = cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
depth_frame = 0
# depthimg = cv2.resize(depthimg, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

#define reference point (center of the frame)
h,w,_ = image.shape
reference_point = ([w/2,h/2])

colors = [(255,0,100), (120,100,255)]

while True:

        #get trackbars values
        y_l_h = cv2.getTrackbarPos("L-H", "Yellow Cone")
        y_l_s = cv2.getTrackbarPos("L-S", "Yellow Cone")
        y_l_v = cv2.getTrackbarPos("L-V", "Yellow Cone")
        y_u_h = cv2.getTrackbarPos("U-H", "Yellow Cone")
        y_u_s = cv2.getTrackbarPos("U-S", "Yellow Cone")
        y_u_v = cv2.getTrackbarPos("U-V", "Yellow Cone")

        r_l_h = cv2.getTrackbarPos("L-H", "Red Cone")
        r_l_s = cv2.getTrackbarPos("L-S", "Red Cone")
        r_l_v = cv2.getTrackbarPos("L-V", "Red Cone")
        r_u_h = cv2.getTrackbarPos("U-H", "Red Cone")
        r_u_s = cv2.getTrackbarPos("U-S", "Red Cone")
        r_u_v = cv2.getTrackbarPos("U-V", "Red Cone")

        #yellow hsv range
        low_yellow = np.array([y_l_h, y_l_s, y_l_v])
        high_yellow = np.array([y_u_h, y_u_s, y_u_v])

        #red hsv range
        low_red = np.array([r_l_h, r_l_s, r_l_v])
        high_red = np.array([r_u_h, r_u_s, r_u_v])

        #create masks of red and yellow cones seperately
        mask_yellow = hsv_fct.create_mask(hsv, low_yellow, high_yellow)
        mask_red = hsv_fct.create_mask(hsv, low_red, high_red)
        mask_red = mask_red - mask_yellow

        cv2.imshow("mask yellow", mask_yellow)
        cv2.imshow("mask red", mask_red)

        #detect for both cone
        image_for_both = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        detected_both = image_for_both
        #cv2.resize(image_for_both, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
        advanced = image_for_both
        #cv2.resize(image_for_both, (network_width, network_height), interpolation=cv2.INTER_LINEAR)

        #detect for yellow cone
        detected_both, detected_yellow, yolo_yellow_cones_center, yellow_detections = yolo_fct.cone_detect(detected_both, image, mask_yellow, net, (255,0,100), "Yellow Cone")
        #detect for red cone
        detected_both, detected_red, yolo_red_cones_center, red_detections = yolo_fct.cone_detect(detected_both, image, mask_red, net, (120,100,255), "Red Cone")

        #get depth data from depthmap
        advanced, center_with_depth_yellow = acc.getDepth_dcm(yellow_detections, depth_frame, depthimg, advanced, colors)
        advanced, center_with_depth_red = acc.getDepth_dcm(red_detections, depth_frame, depthimg, advanced, colors)

        #connect the yellow & red cone centers 
        # detected_both, detected_yellow, yolo_slopes_yellow = yolo_fct.cone_connect(detected_yellow, detected_both, yolo_yellow_cones_center)
        # detected_both, detected_red, yolo_slopes_red = yolo_fct.cone_connect(detected_red, detected_both, yolo_red_cones_center)
        #connect on the advanced img (with depth)
        advanced, detected_yellow, yolo_slopes_yellow = acc.cone_connect_with_depth(advanced, center_with_depth_yellow, 0)
        advanced, detected_red, yolo_slopes_red = acc.cone_connect_with_depth(advanced, center_with_depth_red, 1)
        print("yellow slopes: ", yolo_slopes_yellow[0])
        print("red slopes: ", yolo_slopes_red[0])

        #resize the result images
        # detected_yellow = cv2.resize(detected_yellow, (w, h), interpolation=cv2.INTER_LINEAR)
        # detected_red = cv2.resize(detected_red, (w, h), interpolation=cv2.INTER_LINEAR)
        # detected_both = cv2.resize(detected_both, (w, h), interpolation=cv2.INTER_LINEAR)
        # advanced = cv2.resize(advanced, (w, h), interpolation=cv2.INTER_LINEAR)

        #directions
        directions = acc.get_directions_array(yolo_slopes_yellow, yolo_slopes_red)
        print("directions: ", directions)

        cv2.imshow("img", image)
        # cv2.imshow("overall detection ",detected_both)
        cv2.imshow("advanced", advanced)

        key = cv2.waitKey(25)
        if key == 27:
                break

cv2.destroyAllWindows()