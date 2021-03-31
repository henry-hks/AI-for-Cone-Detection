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
sys.path.append("/home/fyp/darknet")
import darknet as dn 
import darknet_images as dni 

sys.path.append("/home/fyp/function")
import hsv_fct, yolo_fct
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
COM_PORT = '/dev/ttyTHS1'
BAUD_RATES = 115200
ser = serial.Serial(COM_PORT, BAUD_RATES)

apex_coor_array = []

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
                image = np.asanyarray(color_frame.get_data())
                depthimg = np.asanyarray(depth_frame.get_data())
                depthimg_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depthimg, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("depth color map", depthimg_colormap)

                #get image and depthimg
                # time_for_cap = 0
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
                # y_l_s = 187
                # y_l_v = 135
                # y_u_h = 78
                # y_u_s = 255
                # y_u_v = 255

                # r_l_h = 0
                # r_l_s = 197
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
                # image_for_both = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image_for_both = image
                # detected_both = image_for_both
                # cv2.resize(image_for_both, (network_width, network_height), interpolation=cv2.INTER_LINEAR)
                advanced = image
                image_board = image
                # cv2.resize(image_for_both, (network_width, network_height), interpolation=cv2.INTER_LINEAR)

                #detect for both (depth yolo)
                # both_detections = yolo_fct.depth_cone_detect(depthimg_colormap, net, "Cone")

                #detect for yellow cone
                yellow_detections = yolo_fct.cone_detect(image_rgb, mask_yellow, net, (255,0,100), "Yellow Cone")
                # yellow_detections = yolo_fct.cone_detect(image, mask_yellow, net, (255,0,100), "Yellow Cone")
                # detected_both, detected_yellow, yolo_yellow_cones_center, yellow_detections = yolo_fct.cone_detect(detected_both, image, mask_yellow, net, (255,0,100), "Yellow Cone")
                
                #detect for red cone
                red_detections = yolo_fct.cone_detect(depthimg_colormap, mask_red, net, (120,100,255), "Red Cone")
                # red_detections = yolo_fct.cone_detect(image, mask_red, net, (120,100,255), "Red Cone")
                # detected_both, detected_red, yolo_red_cones_center, red_detections = yolo_fct.cone_detect(detected_both, image, mask_red, net, (120,100,255), "Red Cone")

                #get depth data from depthmap
                advanced, center_with_depth_yellow = acc.getDepth_dcm(yellow_detections, depth_frame, depthimg, advanced, colors)
                advanced, center_with_depth_red = acc.getDepth_dcm(red_detections, depth_frame, depthimg, advanced, colors)

                #connect the yellow & red cone centers 
                # detected_both, detected_yellow, yolo_slopes_yellow = yolo_fct.cone_connect(detected_yellow, detected_both, yolo_yellow_cones_center)
                # detected_both, detected_red, yolo_slopes_red = yolo_fct.cone_connect(detected_red, detected_both, yolo_red_cones_center)
                #connect on the advanced img (with depth)
                # wakakakaka bOBOBOBOBObobobobobbobboolubbaouforever la bobobobooooobobobobobobobob
                advanced, yellow_cone_connect_sequence = acc.cone_connect_with_depth(advanced, center_with_depth_yellow, 0)
                advanced, red_cone_connect_sequence = acc.cone_connect_with_depth(advanced, center_with_depth_red, 1)
                # print("slope yellow: ", yolo_slopes_yellow)
                # print("slope red: ", yolo_slopes_red)
                # print("real coor yellow: ", yellow_cone_connect_sequence)
                # print("real coor red: ", red_cone_connect_sequence)

                #detect direction
                directions = acc.simple_direction_detect(yellow_cone_connect_sequence, red_cone_connect_sequence)
                # print("directions: ", directions)

                #detect apex
                # apex_coor = acc.apex_detect(advanced, yellow_cone_connect_sequence, red_cone_connect_sequence, yolo_slopes_yellow, yolo_slopes_red, directions)
                apex_coor = acc.simple_apex_detect(advanced, yellow_cone_connect_sequence, red_cone_connect_sequence)
                
                # apex_coor_array.append(apex_coor)
                
                # if count > 4:
                #         if len(apex_coor_array) >= 2:
                #                 apex_coor_sure = apex_coor_array[len(apex_coor_array)-1][0]
                #                 apex_coor_array = []
                
                # if apex_coor_sure:
                # # print("apex coor ", apex_coor[0][0][0])
                # cv2.drawMarker(detected_both, (apex_coor[0][0][0], apex_coor[0][0][1]), (0,200,200), cv2.MARKER_CROSS, 10,1,1)
                # cv2.putText(detected_both, "APEX", (apex_coor[0][0][0], apex_coor[0][0][1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,10), 2)
                
                height, width, _ = advanced.shape

                #main control algorithm

                go_to_second_layer = 0
                #first layer (safety net)
                if not yellow_detections or not red_detections: #no cones detected
                        print("Action: STOP") #stop the car

                elif yellow_cone_connect_sequence: #yellow cone detected
                        if yellow_cone_connect_sequence[0][1] <= 60 and yellow_cone_connect_sequence[0][2][0] >= -0.02: #car crashing into the yellow cone
                                print("Turn Right Now!") #turn right immediately
                        go_to_second_layer = 1

                elif red_cone_connect_sequence: #red cond detected
                        if red_cone_connect_sequence[0][1] <= 60 and red_cone_connect_sequence[0][2][0] <= 0.02: #car crashing into the red cone
                                print("Turn Left Now!") #turn left immediately
                        go_to_second_layer = 1

                else:
                        cv2.arrowedLine(advanced, (int(width/2), height-20), (int(width/2), int(height/2)), (10,255,10), 2)
                        print("Move Straight!")


                #second layer (speed enhance/ cutting corner)
                if go_to_second_layer == 1:
                        if apex_coor: #if apex detected
                                servo_adjust = round(mc.steering_control(apex_coor), 3) #find the servo pitch to be adjust (min:-15; max:15)
                                speed_adjust = round(mc.speed_control(apex_coor), 3) #find the speed needed
                                if servo_adjust > 0: # turn left
                                        print("Turn Left")
                                        cv2.arrowedLine(advanced, (int(width/2), height-20), (apex_coor[0][0][0]+20, apex_coor[0][0][1]), (10,255,10), 2)
                                if servo_adjust < 0: # turn right
                                        print("Turn Right")
                                        cv2.arrowedLine(advanced, (int(width/2), height-20), (apex_coor[0][0][0]-20, apex_coor[0][0][1]), (10,255,10), 2)

                                print("servo_adjust: ", servo_adjust)
                                print("speed needed: ", speed_adjust)
                                cv2.putText(advanced, "servo adjust: {}".format(servo_adjust), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
                                cv2.putText(advanced, "speed: {}".format(speed_adjust), (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
                                #message format: [F/B] [PWM] [+/-] [servo_pitch]
                                #seperate the sign from the servo pitch (+:1, -:0)
                                servo_sign = np.sign(servo_adjust)
                                if servo_sign == 1 or servo_sign == 0:
                                        sign = 1
                                elif servo_sign == -1:
                                        sign == 0

                                #send message 
                                message = "F{}{}{}".format(speed_adjust, sign, abs(servo_adjust))
                                ser.write(message.encode())

                #resize the result images
                # detected_yellow = cv2.resize(detected_yellow, (w, h), interpolation=cv2.INTER_LINEAR)
                # detected_red = cv2.resize(detected_red, (w, h), interpolation=cv2.INTER_LINEAR)
                # detected_both = cv2.resize(detected_both, (w, h), interpolation=cv2.INTER_LINEAR)
                # advanced = cv2.resize(advanced, (w, h), interpolation=cv2.INTER_LINEAR)
                
                fps = round(1 / (time.time() - start_time), 3)
                # cv2.putText(advanced,"FPS: {}".format(fps), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
                start_time = time.time()
                # cv2.imshow("img", image)
                # cv2.imshow("overall detection ",detected_both)
                cv2.imshow("advanced", advanced)

                #capture result
                time_for_capture = 0
                k = cv2.waitKey(33)
                if k == ord('a'): #press a to cap
                        time_for_capture = 1
                        if time_for_capture == 1:
                                time_for_capture = 0
                                path_dir = "/home/fyp/result_cap_image/"
                                cv2.imwrite(os.path.join(path_dir, "image.jpg"), advanced)
                                cv2.imwrite(os.path.join(path_dir, "depthimg_colormap.png"), depthimg_colormap)
                                cv2.imwrite(os.path.join(path_dir, "mask_yellow.jpg"), mask_yellow)
                                cv2.imwrite(os.path.join(path_dir, "mask_red.jpg"), mask_yellow)


                # Press esc or 'q' to close the image window
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                        cv2.destroyAllWindows()
                        break
finally:
        # Stop streaming
        pipeline.stop()
