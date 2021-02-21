import ctypes
import math
import random
import os
import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import time
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
cv2.createTrackbar("L-V", "Yellow Cone", 135, 255, nothing)
cv2.createTrackbar("U-H", "Yellow Cone", 78, 180, nothing)
cv2.createTrackbar("U-S", "Yellow Cone", 255, 255, nothing)
cv2.createTrackbar("U-V", "Yellow Cone", 255, 255, nothing)

cv2.namedWindow("Red Cone")
cv2.createTrackbar("L-H", "Red Cone", 0, 180, nothing)
cv2.createTrackbar("L-S", "Red Cone", 197, 255, nothing)
cv2.createTrackbar("L-V", "Red Cone", 110, 255, nothing)
cv2.createTrackbar("U-H", "Red Cone", 180, 180, nothing)
cv2.createTrackbar("U-S", "Red Cone", 255, 255, nothing)
cv2.createTrackbar("U-V", "Red Cone", 255, 255, nothing)

#read image and depthmap
# path_img = "/home/fyp/Vincent/darknet/data/img/72.jpg"
# path_dep = "/home/fyp/Vincent/darknet/data/depthmap/d72.png"
# image = cv2.imread(path_img)
# depthimg = cv2.imread(path_dep)
# # image = cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
# depthimg = cv2.resize(depthimg, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
# image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

#get image from realsense
device_id = '036522072893'
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(device_id)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)  # depth
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30) # rgb
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63) # acceleration
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)  # gyroscope
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)
align_to = rs.stream.color
align = rs.align(align_to)
#color for drawing boundary box
#colors = darknet.class_colors(ClassNameMain)

try:
        frame_count = 0
        start_time = time.time()

        while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                frame_time = time.time() - start_time
                frame_count += 1
                
                # get imu frames
                accel_frame = frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)
                gyro_frame = frames.first_or_default(rs.stream.gyro, rs.format.motion_xyz32f)
                
                # Align the depth frame to color frame
                aligned_frames = align.process(frames) 
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # Convert images to numpy arrays
                depthimg = np.asanyarray(depth_frame.get_data())
                image = np.asanyarray(color_frame.get_data())


                # depthimg = cv2.resize(depthimg_ori, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

                h,w,_ = image.shape
                reference_point = ([w/2,h/2])

                colors = [(255,0,100), (120,100,255)]
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
                # cv2.resize(image_for_both, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
                advanced = image_for_both
                # cv2.resize(image_for_both, (network_width, network_height), interpolation=cv2.INTER_LINEAR)

                #detect for yellow cone
                detected_both, detected_yellow, yolo_yellow_cones_center, yellow_detections = yolo_fct.cone_detect(detected_both, image, mask_yellow, net, (255,0,100), "Yellow Cone")
                #detect for red cone
                detected_both, detected_red, yolo_red_cones_center, red_detections = yolo_fct.cone_detect(detected_both, image, mask_red, net, (120,100,255), "Red Cone")

                #get depth data from depthmap
                advanced, center_with_depth_yellow = acc.getDepth_dcm(yellow_detections, depth_frame, depthimg, advanced, colors)
                advanced, center_with_depth_red = acc.getDepth_dcm(red_detections, depth_frame, depthimg, advanced, colors)

                #connect the yellow & red cone centers 
                detected_both, detected_yellow, yolo_slopes_yellow = yolo_fct.cone_connect(detected_yellow, detected_both, yolo_yellow_cones_center)
                detected_both, detected_red, yolo_slopes_red = yolo_fct.cone_connect(detected_red, detected_both, yolo_red_cones_center)
                #connect on the advanced img (with depth)
                advanced, yellow_cone_connect_sequence, yolo_slopes_yellow = acc.cone_connect_with_depth(advanced, center_with_depth_yellow, 0)
                advanced, red_cone_connect_sequence, yolo_slopes_red = acc.cone_connect_with_depth(advanced, center_with_depth_red, 1)
                print("slope yellow: ", yolo_slopes_yellow)
                print("slope red: ", yolo_slopes_red)
                

                #fuzzy logic
                directions = acc.get_directions_array(yolo_slopes_yellow, yolo_slopes_red)
                print("direction yellow: ", directions)

                #resize the result images
                # detected_yellow = cv2.resize(detected_yellow, (w, h), interpolation=cv2.INTER_LINEAR)
                # detected_red = cv2.resize(detected_red, (w, h), interpolation=cv2.INTER_LINEAR)
                # detected_both = cv2.resize(detected_both, (w, h), interpolation=cv2.INTER_LINEAR)
                # advanced = cv2.resize(advanced, (w, h), interpolation=cv2.INTER_LINEAR)
                
                
                fps = 1 / (time.time() - start_time)
                cv2.putText(advanced,"FPS: {}".format(fps), (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
                start_time = time.time()
                cv2.imshow("img", image)
                cv2.imshow("overall detection ",detected_both)
                cv2.imshow("advanced", advanced)

                
                # Press esc or 'q' to close the image window
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                        cv2.destroyAllWindows()
                        break
finally:
        # Stop streaming
        pipeline.stop()

