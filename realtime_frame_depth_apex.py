import ctypes
import math
import random
import os
import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import time
import serial
import paho.mqtt.client as mqtt
sys.path.append("/media/fyp/3366-6130/darknet")
import darknet as dn 
import darknet_images as dni 

client = mqtt.Client()
client.username_pw_set("try", "xxxx")
client.connect("192.168.31.241", 1883, 60)
# client.connect("192.168.0.161", 1883, 60)  #tp-link

sys.path.append("/home/fyp/function")
import hsv_fct, yolo_fct, maptv
import adv_cone_connect as acc 
import motion_control as mc 
# import system_filter as sf 

#load yolov4 network
net, class_names, class_colors = dn.load_network("/home/fyp/gazebo_ws/src/robot_vision/src/yolov4_cone/yolov4_cones.cfg","/home/fyp/gazebo_ws/src/robot_vision/src/yolov4_cone/cones.data","/home/fyp/gazebo_ws/src/robot_vision/src/yolov4_cone/yolov4_cones.weights",batch_size=1)
# net, class_names, class_colors = dn.load_network("/media/fyp/HenrySSD/Polyu_EIE/FYP/new_weight/yolov4-conesDCM.cfg","/media/fyp/HenrySSD/Polyu_EIE/FYP/new_weight/conesdcm.data","/media/fyp/HenrySSD/Polyu_EIE/FYP/new_weight/yolov4-conesDCM_best.weights")

# class_names = ["Yellow_Cone","Red_Cone"]
network_width = dn.network_width(net)
network_height = dn.network_height(net)
yolo_rp = ([network_width/2, network_height/2])

#Trackbar configuration
# def nothing(x):
#     pass

# cv2.namedWindow("Yellow Cone")
# cv2.createTrackbar("L-H", "Yellow Cone", 4, 180, nothing)
# cv2.createTrackbar("L-S", "Yellow Cone", 187, 255, nothing)
# cv2.createTrackbar("L-V", "Yellow Cone", 135, 255, nothing)
# cv2.createTrackbar("U-H", "Yellow Cone", 78, 180, nothing)
# cv2.createTrackbar("U-S", "Yellow Cone", 255, 255, nothing)
# cv2.createTrackbar("U-V", "Yellow Cone", 255, 255, nothing)

# cv2.namedWindow("Red Cone")
# cv2.createTrackbar("L-H", "Red Cone", 0, 180, nothing)
# cv2.createTrackbar("L-S", "Red Cone", 197, 255, nothing)
# cv2.createTrackbar("L-V", "Red Cone", 110, 255, nothing)
# cv2.createTrackbar("U-H", "Red Cone", 180, 180, nothing)
# cv2.createTrackbar("U-S", "Red Cone", 255, 255, nothing)
# cv2.createTrackbar("U-V", "Red Cone", 255, 255, nothing)

#read image and depthmap
# path_img = "/home/fyp/Vincent/darknet/data/img/72.jpg"
# path_dep = "/home/fyp/Vincent/darknet/data/depthmap/d72.png"
# image = cv2.imread(path_img)
# depthimg = cv2.imread(path_dep)
# # image = cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
# depthimg = cv2.resize(depthimg, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
# image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

#realsense configuration
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

#serial output
COM_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
# COM_PORT = '/dev/ttyTHS2'
BAUD_RATES = 115200
# BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES)
offset_value = 4
left_right = 'L'
offset_message = 'O{}{}\n'.format(left_right, offset_value)
ser.write(offset_message.encode())

slope_array = []
angle_dists_array = []
ada_memory = []
speed_array = []
servo_array = []
counter = 0
first_run = 1
sos_counter = 0
camera_coor = [[0,0],0,[0,0,0]]

try:
        frame_count = 0
        start_time = time.time()

        while True:
                width = 300
                height = 450
                bg_mid_x = int(width/2)
                starting_height = height - 100

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
                image = np.asanyarray(color_frame.get_data())
                depthimg = np.asanyarray(depth_frame.get_data())
                depthimg_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depthimg, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("depth color map", depthimg_colormap)

                #get image and depthimgtrans_pt = []
                # time_for_cap = 0trans_pt = []
                # k = cv2.waitKey(33)
                # if k == ord('g'): #press g to cap
                #         if time_for_cap == 1:
                #                 time_for_cap = 0
                #                 path_dir = "/home/result_cap_image/"
                #                 cv2.imwrite(os.path.join(path_dir, "image.jpg"), image)
                #                 cv2.imwrite(os.path.join(path_dir, "depth.png"), depthimg)
                #                 cv2.imwrite(os.path.join(path_dir, "depthimg_colormap.png"), depthimg_colormap)

                # depthimg = cv2.resize(depthimg_ori, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)

                h,w,_ = image.shape
                reference_point = ([w/2,h/2])

                colors = [(255,0,100), (120,100,255)]
                #get trackbars values
                # y_l_h = cv2.getTrackbarPos("L-H", "Yellow Cone")
                # y_l_s = cv2.getTrackbarPos("L-S", "Yellow Cone")
                # y_l_v = cv2.getTrackbarPos("L-V", "Yellow Cone")
                # y_u_h = cv2.getTrackbarPos("U-H", "Yellow Cone")
                # y_u_s = cv2.getTrackbarPos("U-S", "Yellow Cone")
                # y_u_v = cv2.getTrackbarPos("U-V", "Yellow Cone")

                # r_l_h = cv2.getTrackbarPos("L-H", "Red Cone")
                # r_l_s = cv2.getTrackbarPos("L-S", "Red Cone")
                # r_l_v = cv2.getTrackbarPos("L-V", "Red Cone")
                # r_u_h = cv2.getTrackbarPos("U-H", "Red Cone")
                # r_u_s = cv2.getTrackbarPos("U-S", "Red Cone")
                # r_u_v = cv2.getTrackbarPos("U-V", "Red Cone")

                #configure for ihome
                # y_l_h = 4
                # y_l_s = 150
                # y_l_v = 135
                # y_u_h = 78
                # y_u_s = 255
                # y_u_v = 255

                # r_l_h = 0
                # r_l_s = 150
                # r_l_v = 110
                # r_u_h = 180
                # r_u_s = 255
                # r_u_v = 255

                #config for w402c
                y_l_h = 4
                y_l_s = 91
                y_l_v = 135
                y_u_h = 78
                y_u_s = 255
                y_u_v = 255

                r_l_h = 0
                r_l_s = 91
                r_l_v = 110
                r_u_h = 180
                r_u_s = 255
                r_u_v = 255

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

                # cv2.imshow("mask yellow", mask_yellow)
                # cv2.imshow("mask red", mask_red)

                #detect for both cone
                advanced = image

                #detect for yellow cone
                yellow_detections = []
                yellow_detections, masked_yellow = yolo_fct.cone_detect(image_rgb, mask_yellow, net, (255,0,100), "Yellow Cone")
                # print("yellow detections: ", yellow_detections)
                # yellow_detections = yellow_detections[0]
                # yellow_detections = yolo_fct.cone_detect(image, mask_yellow, net, (255,0,100), "Yellow Cone")
                # detected_both, detected_yellow, yolo_yellow_cones_center, yellow_detections = yolo_fct.cone_detect(detected_both, image, mask_yellow, net, (255,0,100), "Yellow Cone")
                
                #detect for red cone
                red_detections = []
                red_detections, masked_red = yolo_fct.cone_detect(depthimg_colormap, mask_red, net, (120,100,255), "Red Cone")
                # print("red detections: ", red_detections)
                # red_detections = red_detections[0]
                # red_detections = yolo_fct.cone_detect(image, mask_red, net, (120,100,255), "Red Cone")
                # detected_both, detected_red, yolo_red_cones_center, red_detections = yolo_fct.cone_detect(detected_both, image, mask_red, net, (120,100,255), "Red Cone")

                #get depth data from depthmap
                center_with_depth_yellow = []
                center_with_depth_red = []
                yellow_cone_connect_sequence = []
                red_cone_connect_sequence = []
                if yellow_detections:
                        advanced, center_with_depth_yellow = acc.getDepth_dcm(yellow_detections, depth_frame, depthimg, advanced, colors)
                        if len(center_with_depth_yellow) > 4:
                                center_with_depth_yellow[:4]
                if red_detections:
                        advanced, center_with_depth_red = acc.getDepth_dcm(red_detections, depth_frame, depthimg, advanced, colors)
                        if len(center_with_depth_red) > 4:
                                center_with_depth_red[:4]
                #project the raw cones to the top-view map
                # top_view_map_raw, yellow_cone_tv_coors, red_cone_tv_coor = maptv.get_map(center_with_depth_yellow, center_with_depth_red)
                
                #connect the yellow & red cone centers (2D)
                # detected_both, detected_yellow, yolo_slopes_yellow = yolo_fct.cone_connect(detected_yellow, detected_both, yolo_yellow_cones_center)
                # detected_both, detected_red, yolo_slopes_red = yolo_fct.cone_connect(detected_red, detected_both, yolo_red_cones_center)
                
                #connect on the advanced img (with depth)
                advanced, yellow_cone_connect_sequence = acc.cone_connect_with_depth(advanced, center_with_depth_yellow, 0)
                advanced, red_cone_connect_sequence = acc.cone_connect_with_depth(advanced, center_with_depth_red, 1)

                #project the cones sequence to another top-view map
                # top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv = maptv.get_map(yellow_cone_connect_sequence, red_cone_connect_sequence)
                # maptv.connect_on_map(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv)

                # print("real coor yellow: ", yellow_cone_connect_sequence)
                # print("real coor red: ", red_cone_connect_sequence)

                #detect direction
                # if yellow_cone_connect_sequence and red_cone_connect_sequence:
                #         directions = acc.simple_direction_detect(yellow_cone_connect_sequence, red_cone_connect_sequence)
                # print("directions: ", directions)

                #detect apex
                # apex_coor = acc.apex_detect(advanced, yellow_cone_connect_sequence, red_cone_connect_sequence, yolo_slopes_yellow, yolo_slopes_red, directions)
                # apex_coor, side = acc.simple_apex_detect(advanced, yellow_cone_connect_sequence, red_cone_connect_sequence)
                
                # apex_coor_array.append(apex_coor)
                
                # if count > 4:
                #         if len(apex_coor_array) >= 2:
                #                 apex_coor_sure = apex_coor_array[len(apex_coor_array)-1][0]
                #                 apex_coor_array = []
                
                # if apex_coor_sure:
                # # print("apex coor ", apex_coor[0][0][0])
                # cv2.drawMarker(detected_both, (apex_coor[0][0][0], apex_coor[0][0][1]), (0,200,200), cv2.MARKER_CROSS, 10,1,1)
                # cv2.putText(detected_both, "APEX", (apex_coor[0][0][0], apex_coor[0][0][1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,10), 2)
                
                # height, width, _ = advanced.shape

                #main control algorithm
                go_to_second_layer = 0
                servo_adjust = 0
                servo_adjust_round_10 = 0
                speed = 0
                angle = 0
                distance1 = 0
                distance2 = 0
                angle_fc = -999
                distance_fc = -999
                x_coor = 0
                depth = 0
                apex_coor = []
                trans_pt = 0
                next_point = ()
                yccs_tv = []
                rccs_tv = []
                # top_view_map, servo_adjust, speed = maptv.instance_map(center_with_depth_yellow, center_with_depth_red, yellow_cone_connect_sequence, red_cone_connect_sequence, apex_coor, side)
                
                _, yccs_tv, rccs_tv = maptv.get_map(yellow_cone_connect_sequence, red_cone_connect_sequence)
                top_view_map, cwdy_tv, cwdy_tv = maptv.get_map(center_with_depth_yellow, center_with_depth_red)
                top_view_map_raw = top_view_map
                if yellow_cone_connect_sequence or red_cone_connect_sequence:
                        
                        if len(yccs_tv) == 3 or len(rccs_tv) ==3:
                                maptv.connect_on_map(top_view_map, yccs_tv, rccs_tv)
                        if len(yccs_tv) == 3 and len(rccs_tv) ==3:
                                path_pt_0, path_pt_1, path_pt_2 = maptv.get_mids(top_view_map, yccs_tv, rccs_tv)
                        # maptv.get_better_path(top_view_map, yccs_tv, rccs_tv)
                        # _ = maptv.get_better_path_2(top_view_map, yccs_tv, rccs_tv)
                # else:
                #         top_view_map_raw, cwdy_tv, cwdy_tv = maptv.get_map(center_with_depth_yellow, center_with_depth_red)
                        
                #         # top_view_map_raw = top_view_map
                #         maptv.connect_on_map(top_view_map_raw, cwdy_tv, cwdy_tv)

                # layer 0 (safety net)
                depth_thres = 100
                x_thres = 0.15
                if not yellow_detections and not red_detections: #no cones detected
                        print("No Cone Detected! Action: STOP / EXPLORE") #stop the car
                        angle = 999
                        distance1 = -999
                        sos_counter += 1
                        # servo_adjust = 0
                        # speed = 0
                
                elif center_with_depth_yellow or center_with_depth_red:
                        sos_counter = 0
                        if center_with_depth_yellow:
                                        # servo_adjust = -15
                                        # speed = 5
                                if yellow_cone_connect_sequence: #yellow cone detected
                                        # if yellow_cone_connect_sequence[0][1] <= 60 and yellow_cone_connect_sequence[0][2][0] >= -0.02: #car crashing into the yellow cone
                                        #         print("Turn Right Now!") #turn right immediately
                                        go_to_second_layer += 1

                        if center_with_depth_red:
                                
                                        # servo_adjust = 15
                                        # speed = 5
                                if red_cone_connect_sequence: #red cond detected
                                        # if red_cone_connect_sequence[0][1] <= 60 and red_cone_connect_sequence[0][2][0] <= 0.02: #car crashing into the red cone
                                        #         print("Turn Left Now!") #turn left immediately
                                        go_to_second_layer += 1

                # layer 1 (basic movement)
                if go_to_second_layer == 0 :
                        # cones detected
                        # but no ccs and specified detected
                        if center_with_depth_yellow and not center_with_depth_red:
                                # only yellow cones detected
                                # if center_with_depth_yellow[0][1] <= depth_thres and center_with_depth_yellow[0][2][0] >= -x_thres: #car crashing into the yellow cone
                                #         print("Turn Right Now!") #turn right immediately
                                angle = 60
                                distance = 50
                                        
                                # if len(center_with_depth_yellow) >= 2:
                                #         slope_yellow = maptv.get_slope(center_with_depth_yellow[0], center_with_depth_yellow[1])
                                #         if slope_yellow >= 1:
                                #                 angle, distance1, x_coor, depth = mc.immediate_motion_needed_2(center_with_depth_yellow[0], center_with_depth_yellow[1]) # -2 for only yellow cone detected
                                #         else:
                                #                 angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_yellow[0], -2)
                                # else:
                                #         angle = 55
                                #         distance = 80

                        elif center_with_depth_red and not center_with_depth_yellow:
                                # only red cones detected
                                # if len(center_with_depth_red) >= 2:
                                #         slope_red = maptv.get_slope(center_with_depth_red[0], center_with_depth_red[1])
                                #         if slope_red <= -1:
                                #                 angle, distance1, x_coor, depth = mc.immediate_motion_needed_2(center_with_depth_red[0], center_with_depth_red[1]) # -2 for only yellow cone detected
                                #         else:
                                #                 angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_red[0], 2) # 2 for only red cone detected
                                # else:
                                #         angle = -55
                                #         distance = 80
                                # if center_with_depth_red[0][1] <= depth_thres and center_with_depth_red[0][2][0] <= x_thres: #car crashing into the yellow cone
                                #         print("Turn Right Now!") #turn right immediately
                                angle = -60
                                distance = 50

                        elif center_with_depth_red and center_with_depth_yellow:
                                # both yellow and red cones detected
                                # calculate the mid point of the first yellow-red cone pair
                                first_cones_midpt = [0,0,[(center_with_depth_yellow[0][2][0]+ center_with_depth_red[0][2][0])/2, 0 , (center_with_depth_yellow[0][2][2]+ center_with_depth_red[0][2][2])/2]]
                                
                                angle, distance1, x_coor, depth = mc.immediate_motion_needed(first_cones_midpt, 0) # 0 for calculate mid point between the first cone pair
                                
                                
                                # if abs(center_with_depth_yellow[0][2][2] - center_with_depth_red[0][2][2]) <= 1:
                                #         #depth difference small enough
                                #         first_cones_midpt = [0,0,[(center_with_depth_yellow[0][2][0]+ center_with_depth_red[0][2][0])/2, 0 , (center_with_depth_yellow[0][2][2]+ center_with_depth_red[0][2][2])/2]]
                                
                                #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(first_cones_midpt, 0) # 0 for calculate mid point between the first cone pair
                                # elif center_with_depth_yellow[0][2][2] < center_with_depth_red[0][2][2]:
                                #         #yellow near when large depth difference
                                #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_yellow[0], -2)
                                # elif center_with_depth_yellow[0][2][2] > center_with_depth_red[0][2][2]:
                                #         #red near when large depth difference
                                #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_red[0], 2)

                # layer 1.5 (basic movement with ccs)
                if go_to_second_layer == 1:
                        # one of the ccs detected
                        # but no specified path detected
                        # drive within the left/ right detected track
                        if yellow_cone_connect_sequence:
                                # slope_yellow = maptv.get_slope(yellow_cone_connect_sequence[0], yellow_cone_connect_sequence[1])
                                # if slope_yellow >= 1:
                                #         angle, distance1, x_coor, depth = mc.immediate_motion_needed_2(yellow_cone_connect_sequence[0], yellow_cone_connect_sequence[1]) # -2 for only yellow cone detected
                                # else:
                                #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(yellow_cone_connect_sequence[0], -2)
                                if yellow_cone_connect_sequence[0][1] <= depth_thres and yellow_cone_connect_sequence[0][2][0] >= -x_thres: #car crashing into the yellow cone
                                        print("Turn Right Now!") #turn right immediately
                                        angle = 35
                                        distance = 50
                                        

                        elif red_cone_connect_sequence:
                                # slope_red = maptv.get_slope(red_cone_connect_sequence[0], red_cone_connect_sequence[1])
                                # if slope_red <= -1:
                                #         angle, distance1, x_coor, depth = mc.immediate_motion_needed_2(red_cone_connect_sequence[0], red_cone_connect_sequence[1]) # -2 for only yellow cone detected
                                # else:
                                #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(red_cone_connect_sequence[0], 2)
                                if red_cone_connect_sequence[0][1] <= depth_thres and red_cone_connect_sequence[0][2][0] <= x_thres: #car crashing into the yellow cone
                                        print("Turn Right Now!") #turn right immediately
                                        angle = -35
                                        distance = 50
                                
                        # servo_adjust = 0
                        # speed = 100

                        # if yellow_cone_connect_sequence and not red_cone_connect_sequence:
                                
                        #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(yellow_cone_connect_sequence[0], -2) # -2 for only yellow cone detected

                        # elif red_cone_connect_sequence and not yellow_cone_connect_sequence:
                                
                        #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(red_cone_connect_sequence[0], 2) # 2 for only red cone detected

                        # elif red_cone_connect_sequence and yellow_cone_connect_sequence:
                        #         # both yellow and red cones detected
                        #         # calculate the mid point of the first yellow-red cone pair
                        #         if abs(yellow_cone_connect_sequence[0][2][2] - red_cone_connect_sequence[0][2][2]) <= 1:
                        #                 #depth difference small enough
                        #                 first_cones_midpt = [0,0,[(yellow_cone_connect_sequence[0][2][0]+ red_cone_connect_sequence[0][2][0])/2, 0 , (yellow_cone_connect_sequence[0][2][2]+ red_cone_connect_sequence[0][2][2])/2]]
                                
                        #                 angle, distance1, x_coor, depth = mc.immediate_motion_needed(first_cones_midpt, 0) # 0 for calculate mid point between the first cone pair
                        #         elif yellow_cone_connect_sequence[0][2][2] < red_cone_connect_sequence[0][2][2]:
                        #                 #yellow near when large depth difference
                        #                 angle, distance1, x_coor, depth = mc.immediate_motion_needed(yellow_cone_connect_sequence[0], -2)
                        #         elif yellow_cone_connect_sequence[0][2][2] >= red_cone_connect_sequence[0][2][2]:
                        #                 #red near when large depth difference
                        #                 angle, distance1, x_coor, depth = mc.immediate_motion_needed(red_cone_connect_sequence[0], 2)

                #second layer (speed enhance/ cutting corner)
                if go_to_second_layer == 2: #both ccs detected
                        side = 0
                        apex_coor, side = acc.simple_apex_detect(advanced, yellow_cone_connect_sequence, red_cone_connect_sequence)
                        if apex_coor: #if apex detected
                                print("apex: ", apex_coor)
                                maptv.apex_on_map(top_view_map, apex_coor, side)
                                
                                angle, distance1, x_coor, depth, next_point, distance2 = mc.motion_needed(apex_coor, side, yellow_cone_connect_sequence, red_cone_connect_sequence)
                                
                                if side == -1:
                                        angle_fc, distance_fc, x_coor_fc, depth_fc = mc.immediate_motion_needed(yellow_cone_connect_sequence[0], side)
                                        if angle_fc > angle: #if the first cone block the way to apex
                                                trans_pt = 1
                                elif side == 1:
                                        angle_fc, distance_fc, x_coor_fc, depth_fc = mc.immediate_motion_needed(red_cone_connect_sequence[0], side)
                                        if angle_fc < angle: #if the first cone block the way to apex
                                                trans_pt = 1
                                                
                                        

                                # if servo_adjust > 0: # turn left
                                #         print("Turn Left")
                                #         cv2.arrowedLine(advanced, (int(width/2), height-20), (apex_coor[0][0][0]+20, apex_coor[0][0][1]), (10,255,10), 2)
                                # if servo_adjust < 0: # turn right
                                #         print("Turn Right")
                                #         cv2.arrowedLine(advanced, (int(width/2), height-20), (apex_coor[0][0][0]-20, apex_coor[0][0][1]), (10,255,10), 2)
                        
                        else:
                                print("Move Straight! ")
                                print("6")
                                # if yccs_tv and rccs_tv:
                                color1 = maptv.distance_2_color(path_pt_0, path_pt_1)
                                color2 = maptv.distance_2_color(path_pt_1, path_pt_2)
                                cv2.arrowedLine(top_view_map, path_pt_0, path_pt_1, color1, 1)
                                cv2.arrowedLine(top_view_map, path_pt_1, path_pt_2, color2, 1)
                                
                                first_cones_midpt = [0,0,[(yellow_cone_connect_sequence[0][2][0]+ red_cone_connect_sequence[0][2][0])/2, 0 , (yellow_cone_connect_sequence[0][2][2]+ red_cone_connect_sequence[0][2][2])/2]]
                                angle, distance1, x_coor, depth = mc.immediate_motion_needed(first_cones_midpt, 0)            
                                        
                angle_raw = round(angle,1)
                distance1_raw = round(distance1,1)
                distance2_raw = round(distance2,1)
                
                
                #avg filter the angle & distance
                frame_number = 2
                if counter <= frame_number:
                        counter += 1
                        angle_dists_array.append([angle_raw, distance1_raw, distance2_raw, x_coor, depth])
                        if ada_memory:
                                ada_memory[0] = ada_memory[1]
                                ada_memory[len(ada_memory)-1] = [angle_raw, distance1_raw, distance2_raw, x_coor, depth]
                        if counter == frame_number:
                                first_run = 0
                                #sort array in ascending order
                                angle_dists_array = sorted(angle_dists_array, key=lambda k:[k[1]])
                                # print("###################################speed_array: ", speed_array)
                                # print("###################################servo array: ", servo_array)
                                print("###############################angle_dists_array: ", angle_dists_array)
                                # servo_adjust = servo_array[2]
                                # speed_array = speed_array[2]
                                if not ada_memory:
                                        ada_memory = angle_dists_array # store the angle_dists_array into an memory array
                                
                                #reset counter
                                counter = 0
                                #clear arrays
                                angle_dists_array = []
                                # servo_array = []
                                # speed_array = []
                        

                if first_run == 0 and len(ada_memory) == frame_number:
                        angle = ada_memory[frame_number - 2][0]
                        distance1 = ada_memory[frame_number - 2][1]
                        distance2 = ada_memory[frame_number - 2][2]
                        x_coor = ada_memory[frame_number - 2][3]
                        depth = ada_memory[frame_number - 2][4]
                        
                        angle_new = ada_memory[frame_number - 1][0]
                        distance1_new = ada_memory[frame_number - 1][1]
                        distance2_new = ada_memory[frame_number - 1][2]
                        x_coor_new = ada_memory[frame_number - 1][3]
                        depth_new = ada_memory[frame_number - 1][4]

                        angle_diff = abs(angle_new - angle)
                        distance1_diff = abs(distance1_new - distance1)
                        
                        if distance1_diff <= 10 and not apex_coor:
                                angle = angle_new
                                distance1 = distance1_new
                                distance2 = distance2_new
                                x_coor = x_coor_new
                                depth = depth_new
                        
                        if angle_diff <= angle_new/2 and apex_coor:
                                angle = angle_new
                                distance1 = distance1_new
                                distance2 = distance2_new
                                x_coor = x_coor_new
                                depth = depth_new
                                
                        # if counter == frame_number - 1:
                        #         # clear the memory array after utilization in counter 4
                        #         ada_memory = []
                        if depth != 0:
                                if trans_pt == 1: #if transfer point exists
                                        color_tp = maptv.distance_2_color((bg_mid_x, starting_height), (int(bg_mid_x + x_coor_fc), int(starting_height - depth_fc)))
                                        color_tpp = maptv.distance_2_color((int(bg_mid_x + x_coor_fc), int(starting_height - depth_fc)), (int(bg_mid_x + x_coor), int(starting_height - depth)))
                                        cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), (int(bg_mid_x + x_coor_fc), int(starting_height - depth_fc)), color_tp, 1)
                                        cv2.arrowedLine(top_view_map, (int(bg_mid_x + x_coor_fc), int(starting_height - depth_fc)), (int(bg_mid_x + x_coor), int(starting_height - depth)), color_tpp, 1)
                                else:
                                        color = maptv.distance_2_color((bg_mid_x, starting_height), (int(bg_mid_x + x_coor), int(starting_height - depth)))
                                        cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), (int(bg_mid_x + x_coor), int(starting_height - depth)), color, 1)
                        if next_point and trans_pt == 0:
                                cv2.putText(top_view_map, "Exit PT",(bg_mid_x + next_point[0] - 30, starting_height - next_point[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,10), 1)
                                color_np = maptv.distance_2_color((int(bg_mid_x + x_coor), int(starting_height - depth)), (bg_mid_x + next_point[0], starting_height - next_point[1]))
                                cv2.arrowedLine(top_view_map, (int(bg_mid_x + x_coor), int(starting_height - depth)), (bg_mid_x + next_point[0], starting_height - next_point[1])  , color_np, 1)
                        
                        # if speed == 0 and (center_with_depth_yellow or center_with_depth_red) and side == 0:
                        #         speed == 10

                        # servo_adjust_raw = round(mc.steering(angle))
                        # speed_raw = round(mc.speed_control(distance1))
                        max_speed_for_servo = 35
                        max_speed = 30
                        min_speed = 28
                        if angle != 999 and speed != -999:
                                servo_adjust = round(mc.steering(angle))
                                servo_adjust_round_10 = round(mc.steering(angle)/10)*10
                                speed = round(mc.speed_control(distance1))
                        elif angle == 999 and speed == -999:
                                servo_adjust = 0
                                servo_adjust_round_10 = 0
                                speed = 0

                        #avg filter the servo adjust and speed
                        # if counter <= 5:
                        #         counter += 1
                        #         servo_array.append(servo_adjust_raw)
                        #         speed_array.append(speed_raw)
                        #         if counter == 5:
                        #                 first_run = 
                        #                 #sort array in ascending order
                        #                 servo_array.sort()
                        #                 speed_array.sort()
                        #                 print("###################################speed_array: ", speed_array)
                        #                 print("###################################servo array: ", servo_array)
                        #                 servo_adjust = servo_array[2]
                        #                 speed_array = speed_array[2]
                                        
                        #                 #reset counter
                        #                 counter = 0
                        #                 #clear arrays
                        #                 servo_array = []
                        #                 speed_array = []
                                        
                                
                        if 0 <= abs(servo_adjust) <= 10 and (center_with_depth_yellow or center_with_depth_red):
                                # if the turning angle is small enough
                                # the car could drive with full speed
                                # speed = 100
                                speed = max_speed
                        if abs(servo_adjust) > 10 and speed <min_speed:
                                # if the turning angle is large
                                # the calculated speed might be small in the above calculation
                                # small speed 10PWM is inserted to move the car slowly while turning
                                speed = min_speed
                        
                        print("angular distance: ", angle)
                        print("distance1: ", distance1)
                        print("distance2: ", distance2)
                        print("servo_adjust: ", servo_adjust)
                        print("speed: ", speed)

                        if not speed_array:
                                speed_array.append(speed)
                        elif len(speed_array) == 2:
                                speed_diff = abs(speed_array[1] - speed_array[0])
                                if speed_diff <= 5:
                                        speed_array[0] = speed_array[1]
                                        speed_array[1] = speed
                                else:
                                        speed_array[1] = speed_array[0]
                        
                        if not servo_adjust:
                                servo_array.append(speed)
                        elif len(servo_array) == 2:
                                servo_diff = abs(servo_array[1] - servo_array[0])
                                if servo_diff <= 5:
                                        servo_array[0] = servo_array[1]
                                        servo_array[1] = servo_adjust
                                else:
                                        servo_array[1] = servo_array[0]

                        if apex_coor:
                                cv2.putText(top_view_map, "Angluar distance of the Apex: {}".format(angle), (5 ,height-55), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                                cv2.putText(top_view_map, "Distance of the apex: {}".format(distance1), (5 ,height-40), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                                cv2.putText(top_view_map, "Distance of exit pt.: {}".format(distance2), (5 ,height-70), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                        # cv2.putText(top_view_map, "Servo Adjust: {}".format(servo_adjust), (5 ,height-25), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                        # cv2.putText(top_view_map, "PWM: {}".format(speed), (5 ,height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)

                        # cv2.putText(advanced, "servo adjust: {}".format(servo_adjust), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
                        # cv2.putText(advanced, "speed: {}".format(speed), (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
                        
                        #message format: [F/B] [PWM] [P/N] [servo_pitch]
                        #seperate the sign from the servo pitch (+:1, -:0)
                        servo_sign = np.sign(servo_adjust)
                        sign = "L"
                        if servo_sign == 1:
                                sign = "R"
                        elif servo_sign == -1:
                                sign == "L"
                        elif servo_sign == 0:
                                sign == "S"
                        print("sign: ", sign)

                        servo_adjust_for_send = "00"
                        servo_adjust_round_10_for_send ="00"
                        if 0 <= abs(servo_adjust) < 10: #single digit
                                servo_adjust_for_send = "0{}".format(abs(servo_adjust))
                        elif 100 > abs(servo_adjust) >= 10: #double digits
                                servo_adjust_for_send = "{}".format(abs(servo_adjust))
                        
                        if 0 <= abs(servo_adjust_round_10) < 10: #single digit
                                servo_adjust_round_10_for_send = "0{}".format(abs(servo_adjust_round_10))
                        elif 100 > abs(servo_adjust_round_10) >= 10: #double digits
                                servo_adjust_round_10_for_send = "{}".format(abs(servo_adjust_round_10))
                        
                        speed_for_send = "0"
                        if 0 <= speed < 10: #single digit
                                speed_for_send = "00{}".format(speed)
                        elif 10 <= speed < 100: #double digits
                                speed_for_send = "0{}".format(speed)
                        elif speed == 100: #100
                                speed_for_send = "{}".format(speed)
                        
                       

                        #print message raw
                        # message = "F{}{}{}".format(speed, sign, abs(servo_adjust))
                        message_raw = "F{}{}{}".format(speed_for_send, sign, servo_adjust_for_send)
                        cv2.putText(top_view_map, "Message: {}".format(message_raw), (bg_mid_x + 50 ,height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                        cv2.putText(top_view_map, "Servo Adjust: {}".format(servo_adjust_for_send), (5 ,height-25), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                        cv2.putText(top_view_map, "PWM: {}".format(speed_for_send), (5 ,height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)

                        print("message: ", message_raw)

                        #send message to stm
                                #pwm -> 0-99
                                #servo -> +- 10, 20, 30, 40
                                #message: 1. FXX/ BXX
                                #         2. LXX/ RXX
                                #         3. SOS
                        
                        
                        #cap the speed for send
                        if speed <= min_speed:
                                speed = min_speed
                        if speed >= max_speed:
                                speed = max_speed
                        
                        #add speed for large servo
                        if abs(servo_adjust) >= 15:
                                speed = max_speed_for_servo


                        
                        #append zero to the message
                        if 0 <= speed < 10: #single digit
                                speed_for_send = "0{}".format(speed)
                        elif 10 <= speed <= 100: #double / triple digits
                                speed_for_send = "{}".format(speed) #force to double digit

                        if servo_adjust_round_10_for_send == 0:
                                sign = "L"

                        message_for_send_speed = "F{}\n".format(speed_for_send)
                        message_for_send_servo = "{}{}\n".format(sign, servo_adjust_for_send)
                        
                        # if speed == 0 and servo_adjust == 0:
                        #         sos_counter += 1
                        # else:
                        #         sos_counter = 0
                        
                        if sos_counter >= 4:
                                #send SOS
                                message_for_send_speed = "SOS\n"
                                print("message for send: \n", message_for_send_speed) 
                                ser.write(message_for_send_speed.encode()) #only send one SOS
                        elif sos_counter < 4:
                                
                                # send normal message
                                ser.write(message_for_send_speed.encode())
                                time.sleep(0.03)
                                ser.write(message_for_send_servo.encode())
                                if sign == 'L':
                                        sign = 'R'
                                elif sign == 'R':
                                        sign = 'L'
                                message_for_send_servo = "{}{}\n".format(sign, servo_adjust_for_send)
                                print("message for send: \nSpeed: ", message_for_send_speed)
                                print("Servo: ", message_for_send_servo)
                        
                        
                        '''
                        #wa
                        client.publish("sensor_msgs/Voltage1", 12)
                        client.publish("sensor_msgs/Voltage2", 12)

                        #if msg.topic == "GetMotorCurrent":
                                # message = "CUL"
                                # ser.write(message.encode())
                                # data = ser.readline()
                                # data = bytes(data)
                                
                        temp = round(np.random.uniform(0.29, 0.33), 2)
                        client.publish("sensor_msgs/Current1", temp)
                        client.publish("sensor_msgs/Current2",temp)

                        #if msg.topic == "GetUltrasonicDistance":
                        message = "USD"
                        ser.write(message.encode())
                        time.sleep(0.03)
                        for i in range(2):
                                data_temp = ser.readline()
                                data_temp = data_temp.decode()
                                #print(data_temp)
                                data = data_temp[-2:]
                                print(data)
                                if i == 0:
                                        client.publish("sensor_msgs/Ultrasonic1", data)
                                elif i == 1:
                                        client.publish("sensor_msgs/Ultrasonic2", data)
                        ser.flush()
                        message = ''
                        data = ''
                        data_temp = ''
                        time.sleep(0.03)
                                
                        #if msg.topic == "GetTemperature":
                        message = "TEM"
                        ser.write(message.encode())
                        time.sleep(0.03)
                        data_temp = ser.readline()
                        data_temp = data_temp.decode()
                        #print(data_temp)
                        data = data_temp[-2:]
                        print(data)
                        client.publish("sensor_msgs/Temperature", data)
                        ser.flush()
                        message = ''
                        time.sleep(0.03)

                        #if msg.topic == "GetHumidty":
                                #message = "HUM"
                                #ser.write(message.encode())
                                #data = ser.readline().decode()
                        if count == 0:
                                count += 1
                                humidity_temp = 4.6
                        elif count == 100:
                                humidity_temp = 4.7
                                count = 0
                        client.publish("sensor_msgs/Humidity", humidity_temp)
                        '''
                print("offset message: ", offset_message)

                #mqtt
                client.loop_start()
                #client.publish("status", 1)
                client.publish("ServoAngle", int(-servo_adjust))
                client.publish("MotorSpeed", int(speed))
                client.loop_stop()

                fps = round(1 / (time.time() - start_time), 3)
                # cv2.putText(advanced,"FPS: {}".format(fps), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                start_time = time.time()
                # cv2.imshow("img", image)
                # cv2.imshow("overall detection ",detected_both)
                
                cv2.imshow("advanced", advanced)
                cv2.imshow("top-view map raw", top_view_map_raw)
                cv2.imshow("top-view map", top_view_map)

                #capture results
                time_for_capture = 0
                k = cv2.waitKey(33)
                if k == ord('a'): #press a to cap
                        time_for_capture = 1
                        if time_for_capture == 1:
                                time_for_capture = 0
                                path_dir = "/home/fyp/result_cap_image/with_tv_changed_message/"
                                cv2.imwrite(os.path.join(path_dir, "image.jpg"), advanced)
                                cv2.imwrite(os.path.join(path_dir, "depthimg_colormap.png"), depthimg_colormap)
                                cv2.imwrite(os.path.join(path_dir, "yellow_mask.jpg"), mask_yellow)
                                cv2.imwrite(os.path.join(path_dir, "red_mask.jpg"), mask_red)
                                cv2.imwrite(os.path.join(path_dir, "masked_yellow.jpg"), masked_yellow)
                                cv2.imwrite(os.path.join(path_dir, "masked_red.jpg"), masked_red)
                                cv2.imwrite(os.path.join(path_dir, "top-view_map_raw.jpg"), top_view_map_raw)
                                cv2.imwrite(os.path.join(path_dir, "top-view_map.jpg"), top_view_map)


                # Press esc or 'q' to close the image window
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                        cv2.destroyAllWindows()
                        break
finally:
        # Stop streaming
        pipeline.stop()
