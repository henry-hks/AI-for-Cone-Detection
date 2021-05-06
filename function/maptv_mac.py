import cv2
import numpy as np 
import math, os
import serial
import adv_cone_connect as acc
import sys
import motion_control as mc 

# serial output
COM_PORT = '/dev/cu.usbmodemFD141'
BAUD_RATES = 115200
# BAUD_RATES = 9600
# ser = serial.Serial(COM_PORT, BAUD_RATES)

width = 300
height = 450
bg_mid_x = int(width/2)
starting_height = height - 100
background = np.zeros((height,width,3), np.uint8)

def nothing(x):
    pass

cv2.namedWindow("Yellow Cones")
cv2.createTrackbar("Y-1x", "Yellow Cones", 50, 200, nothing)
cv2.createTrackbar("Y-2x", "Yellow Cones", 47, 200,  nothing)
cv2.createTrackbar("Y-3x", "Yellow Cones", 34, 200,  nothing)
cv2.createTrackbar("Y-1z", "Yellow Cones", 75, 300,  nothing)
cv2.createTrackbar("Y-2z", "Yellow Cones", 119, 300,  nothing)
cv2.createTrackbar("Y-3z", "Yellow Cones", 200, 300,  nothing)

cv2.namedWindow("Red Cones")
cv2.createTrackbar("R-1x", "Red Cones", 155, 200, nothing)
cv2.createTrackbar("R-2x", "Red Cones", 152, 200,  nothing)
cv2.createTrackbar("R-3x", "Red Cones", 146, 200,  nothing)
cv2.createTrackbar("R-1z", "Red Cones", 92, 300,  nothing)
cv2.createTrackbar("R-2z", "Red Cones", 140, 300,  nothing)
cv2.createTrackbar("R-3z", "Red Cones", 200, 300,  nothing)

def distance_2_color(pt1, pt2):
    #euclidean distance
    distance = math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)
    distance_ratio = distance/180
    print("distance_ratio", distance_ratio)
    color_b = 10
    color_g = distance_ratio*255 + 100
    print("color g", color_g)
    color_r = 300 - distance_ratio*255

    if color_g < 0:
        color_g = 0
    if color_g > 255:
        color_g = 255

    if color_r < 0:
        color_r = 0
    if color_r > 255:
        color_r = 255

    color = [color_b, int(color_g), int(color_r)]
    return color

def get_map(yellow_cone_connect_sequence, red_cone_connect_sequence):
    # generate a black background
    width = 300
    height = 450
    bg_mid_x = int(width/2)
    starting_height = height - 100
    background = np.zeros((height,width,3), np.uint8)
    
    #draw the lines indicating the FOV
    #realsense d435i has 86 fov
    fov = 86
    fov_rad = fov * math.pi/180
    line_end_y = int(bg_mid_x / math.tan(fov_rad/2))

    cv2.line(background, (bg_mid_x,starting_height), (0, line_end_y), (200,200,200), 1)
    cv2.line(background, (bg_mid_x,starting_height), (width, line_end_y), (200,200,200), 1)

    #draw a triangle indicating the position of the camera
    cv2.drawMarker(background, (bg_mid_x, starting_height), (255,255,255), cv2.MARKER_TRIANGLE_UP, markerSize= 20, thickness= 1, line_type=1)
    cv2.putText(background, "Car", (bg_mid_x-15, starting_height+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    #project the real-world coordinates to a top-view plane
    #in the top view, 
    # the x-axis is the 3D x coordinate
    # the y-axis is the 3D z coordinate (depth)
    
    yellow_cone_connect_sequence_tv = []
    red_cone_connect_sequence_tv = []
    if yellow_cone_connect_sequence:
        for i in range(len(yellow_cone_connect_sequence)):
            cone_proj_x_yellow = int(bg_mid_x + 100*yellow_cone_connect_sequence[i][2][0])
            cone_proj_y_yellow = int(starting_height - 100*yellow_cone_connect_sequence[i][2][2])
            background = cv2.circle(background, (cone_proj_x_yellow, cone_proj_y_yellow), radius=5, color=(0,255,255), thickness=-1)
            yellow_cone_connect_sequence_tv.append([cone_proj_x_yellow, cone_proj_y_yellow])

    if red_cone_connect_sequence:
        for i in range(len(red_cone_connect_sequence)):
            cone_proj_x_red = int(bg_mid_x + 100*red_cone_connect_sequence[i][2][0])
            cone_proj_y_red = int(starting_height - 100*red_cone_connect_sequence[i][2][2])
            background = cv2.circle(background, (cone_proj_x_red, cone_proj_y_red), radius=5, color=(0,0,255), thickness=-1)
            red_cone_connect_sequence_tv.append([cone_proj_x_red, cone_proj_y_red])

    return background, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv

def connect_on_map(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
    for i in range(len(yellow_cone_connect_sequence_tv)):
        if i < len(yellow_cone_connect_sequence_tv)-1:
            cv2.line(top_view_map, (yellow_cone_connect_sequence_tv[i][0], yellow_cone_connect_sequence_tv[i][1]) , (yellow_cone_connect_sequence_tv[i+1][0], yellow_cone_connect_sequence_tv[i+1][1]), (255,255,100), 1)
    for i in range(len(red_cone_connect_sequence_tv)):
        if i < len(red_cone_connect_sequence_tv)-1:
            cv2.line(top_view_map, (red_cone_connect_sequence_tv[i][0], red_cone_connect_sequence_tv[i][1]) , (red_cone_connect_sequence_tv[i+1][0], red_cone_connect_sequence_tv[i+1][1]), (255,100,255), 1)
        
    # if len(yellow_cone_connect_sequence) >=3 and len(red_cone_connect_sequence)>=3:
    #     cv2.line(top_view_map, (yellow_cone_connect_sequence_tv[0][0], yellow_cone_connect_sequence_tv[0][1]) , (yellow_cone_connect_sequence_tv[1][0], yellow_cone_connect_sequence_tv[1][1]), (255,255,100), 1)
    #     cv2.line(top_view_map, (yellow_cone_connect_sequence_tv[1][0], yellow_cone_connect_sequence_tv[1][1]) , (yellow_cone_connect_sequence_tv[2][0], yellow_cone_connect_sequence_tv[2][1]), (255,255,100), 1)
    #     cv2.line(top_view_map, (red_cone_connect_sequence_tv[0][0], red_cone_connect_sequence_tv[0][1]) , (red_cone_connect_sequence_tv[1][0], red_cone_connect_sequence_tv[1][1]), (255,100,255), 1)
    #     cv2.line(top_view_map, (red_cone_connect_sequence_tv[1][0], red_cone_connect_sequence_tv[1][1]) , (red_cone_connect_sequence_tv[2][0], red_cone_connect_sequence_tv[2][1]), (255,100,255), 1)

def apex_on_map(top_view_map, apex_coor, side):
    width = 300
    height = 450
    bg_mid_x = int(width/2)
    starting_height = height - 100
    
    apex_coor_tv_x = int(bg_mid_x + 100*apex_coor[0][2][0])
    apex_coor_tv_y = int(starting_height - 100*apex_coor[0][2][2])
    cv2.drawMarker(top_view_map, (apex_coor_tv_x, apex_coor_tv_y), (200,100,200), cv2.MARKER_SQUARE, 20,1,1)
    if side == -1:
        apex = (apex_coor_tv_x - 50, apex_coor_tv_y)
        cv2.putText(top_view_map, "APEX", apex, cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,10), 1)
    elif side == 1:
        apex = (apex_coor_tv_x + 10 , apex_coor_tv_y)
        cv2.putText(top_view_map, "APEX", apex, cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,10), 1)

def get_mids(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
    #try to use cv2.ellipse
    # radius = 
    path_pt_0 = (int((yellow_cone_connect_sequence_tv[0][0]+ red_cone_connect_sequence_tv[0][0])/2) , int((yellow_cone_connect_sequence_tv[0][1]+ red_cone_connect_sequence_tv[0][1])/2))
    path_pt_1 = (int((yellow_cone_connect_sequence_tv[1][0]+ red_cone_connect_sequence_tv[1][0])/2) , int((yellow_cone_connect_sequence_tv[1][1]+ red_cone_connect_sequence_tv[1][1])/2))
    path_pt_2 = (int((yellow_cone_connect_sequence_tv[2][0]+ red_cone_connect_sequence_tv[2][0])/2) , int((yellow_cone_connect_sequence_tv[2][1]+ red_cone_connect_sequence_tv[2][1])/2))

    return path_pt_0, path_pt_1, path_pt_2
        
def get_better_path(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
    if len(yellow_cone_connect_sequence_tv) ==3 and len(red_cone_connect_sequence_tv) == 3:
        path_pt_0 = (int((yellow_cone_connect_sequence_tv[0][0]+ red_cone_connect_sequence_tv[0][0])/2) , int((yellow_cone_connect_sequence_tv[0][1]+ red_cone_connect_sequence_tv[0][1])/2))
        path_pt_1 = (int((yellow_cone_connect_sequence_tv[1][0]+ red_cone_connect_sequence_tv[1][0])/2) , int((yellow_cone_connect_sequence_tv[1][1]+ red_cone_connect_sequence_tv[1][1])/2))
        path_pt_2 = (int((yellow_cone_connect_sequence_tv[2][0]+ red_cone_connect_sequence_tv[2][0])/2) , int((yellow_cone_connect_sequence_tv[2][1]+ red_cone_connect_sequence_tv[2][1])/2))
        
        pt_difference_1 = abs(path_pt_0[0] - path_pt_1[0])
        pt_difference_2 = abs(path_pt_1[0] - path_pt_2[0])
        print("pt difference 1: ", pt_difference_1)
        print("pt difference 2: ", pt_difference_2)

    # if pt_difference_1 - pt_difference_2 <= 15:

def get_slope(real_pt1, real_pt2):
    #x/z
    xdiff = (real_pt1[2][0]) - (real_pt2[2][0])
    zdiff = (real_pt1[2][2]) - (real_pt2[2][2])
    if zdiff != 0:
        slope = round(np.true_divide(xdiff, zdiff, out=None), 5)
    else:
        slope = 999
    
    return slope

def get_slope_tv(cone_connect_sequence_tv):
    yolo_slopes = []
    for i in range(len(cone_connect_sequence_tv)):
        if i != len(cone_connect_sequence_tv)-1:
        # #root depth
        # root_depth = cone_connect_sequence[i][2][2]

        # zdiff = 
        #get the slopes between yellow cones
            xdiff = (cone_connect_sequence_tv[i][1]) - (cone_connect_sequence_tv[i+1][1])
            ydiff = (cone_connect_sequence_tv[i][0]) - (cone_connect_sequence_tv[i+1][0])
            if xdiff != 0:
                sl = round(np.true_divide(ydiff, xdiff, out=None), 5)
            else:
                sl = 999
            yolo_slopes.append(sl)

    
    return yolo_slopes

def get_better_path_2(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
    yellow_slope = get_slope_tv(yellow_cone_connect_sequence_tv)
    red_slope = get_slope_tv(red_cone_connect_sequence_tv)

    #initial values
    side = 999 
    
    print("yellow slope: ", yellow_slope)
    print("red slope: ", red_slope)

    if len(yellow_slope) == 2:
        avg_slope_yellow = (yellow_slope[0] + yellow_slope[1]) / 2
        print("yellow slope avg: ", avg_slope_yellow)

    if len(red_slope) == 2:
        avg_slope_red = (red_slope[0] + red_slope[1]) / 2
        print("red slope avg: ", avg_slope_red)

    if len(yellow_slope) == 2 and len(red_slope) == 2:
        avg_of_avg_slopes = (avg_slope_yellow + avg_slope_red) / 2
        # if avg_of_avg_slopes <= 0:
        print("avg of avg slopes: ", avg_of_avg_slopes)
        if -0.1 < avg_of_avg_slopes < 0.1:
            side = 0 #straight
        elif avg_of_avg_slopes >= 0.1 or red_slope[0] == 999 or red_slope[1] == 999:
            side = -1 #left
        elif avg_of_avg_slopes <= - 0.1 or yellow_slope[0] == 999 or yellow_slope[1] == 999:
            side = 1 #right
        print("better side: ", side)

    return side

def instance_map(center_with_depth_yellow, center_with_depth_red, yellow_cone_connect_sequence, red_cone_connect_sequence, apex_coor, side):
    width = 300
    height = 450
    bg_mid_x = int(width/2)
    starting_height = height - 100
    go_to_second_layer = 0
    
    #initial values
    angle = 0
    distance1 = 0
    distance2 = 0
    servo_adjust = 0
    speed = 0
    x_coor = 0
    depth = 0
    next_point = ()
    
    top_view_map, yccs_tv, rccs_tv = get_map(yellow_cone_connect_sequence, red_cone_connect_sequence)
    connect_on_map(top_view_map, yccs_tv, rccs_tv)
    if len(yccs_tv) == 3:
        path_pt_0, path_pt_1, path_pt_2 = get_mids(top_view_map, yccs_tv, rccs_tv)
    get_better_path(top_view_map, yccs_tv, rccs_tv)
    _ = get_better_path_2(top_view_map, yccs_tv, rccs_tv)

    #first layer
    if not center_with_depth_yellow and not center_with_depth_red:
        print("No Cone Detected! Action: STOP / EXPLORE") #stop the car
        servo_adjust = 0
        speed = 0
    
    elif center_with_depth_yellow or center_with_depth_red:
        if center_with_depth_yellow:
            # if center_with_depth_yellow[0][1] <= 60 and center_with_depth_yellow[0][2][0] >= -0.02: ##car crashing into the yellow cone
                # print("Turn Right NOW!") # turn right immediately
                # angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_yellow[0], 1)

                # servo_adjust = -40
                # speed = 5
            if yellow_cone_connect_sequence: #yellow cone detected
                # if yellow_cone_connect_sequence[0][1] <= 60 and yellow_cone_connect_sequence[0][2][0] >= -0.02: #car crashing into the yellow cone
                    # print("Turn Right Now!") #turn right immediately
                go_to_second_layer += 1
        
        if center_with_depth_red:
            # if center_with_depth_red[0][1] <= 60 and center_with_depth_red[0][2][0] <= 0.02: ##car crashing into the red cone
                # print("Turn left NOW!") # turn left immediately
                # angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_red[0], 1)

                # servo_adjust = 40
                # speed = 5
            if red_cone_connect_sequence: #red cond detected
                # if red_cone_connect_sequence[0][1] <= 60 and red_cone_connect_sequence[0][2][0] <= 0.02: #car crashing into the red cone
                #         print("Turn Left Now!") #turn left immediately
                go_to_second_layer += 1
    if go_to_second_layer == 0 :
            if center_with_depth_yellow and not center_with_depth_red:
                    # only yellow cones detected
                    # prevent crashing into the red cones
                    # if center_with_depth_yellow[0][1] <= 60 and center_with_depth_yellow[0][2][0] >= -0.02: ##car crashing into the yellow cone
                    #         print("Turn Right NOW!") # turn right immediately
                    #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_yellow[0], -2)
                    
                    # else:

                    # drive respect to the first detected yellow cone
                    angle, distance1, x_coor, depth = mc.immediate_motion_needed_2(center_with_depth_yellow[0], center_with_depth_yellow[1]) # -2 for only yellow cone detected

            if center_with_depth_red and not center_with_depth_yellow:
                    # only red cones detected
                    # prevent crashing into the red cones
                    # if center_with_depth_red[0][1] <= 60 and center_with_depth_red[0][2][0] <= 0.02: ##car crashing into the red cone
                    #         print("Turn left NOW!") # turn left immediately
                    #         angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_red[0], 1)

                    # drive respect to the first detected red cone
                    angle, distance1, x_coor, depth = mc.immediate_motion_needed(center_with_depth_red[0], 2) # 2 for only red cone detected

            if center_with_depth_red and center_with_depth_yellow:
                    # both yellow and red cones detected
                    # calculate the mid point of the first yellow-red cone pair
                    first_cones_midpt = [0,0,[(center_with_depth_yellow[0][2][0]+ center_with_depth_red[0][2][0])/2, 0 , (center_with_depth_yellow[0][2][2]+ center_with_depth_red[0][2][2])/2]]
            
                    angle, distance1, x_coor, depth = mc.immediate_motion_needed(first_cones_midpt, 0) # 0 for calculate mid point between the first cone pair

    if go_to_second_layer == 1:
            # one of the ccs detected
            # but no specified path detected
            # drive within the left/ right detected track
            if yellow_cone_connect_sequence:
                    if len(center_with_depth_yellow) >= 2:
                        slope_yellow = get_slope(center_with_depth_yellow[0], center_with_depth_yellow[1])
                        if slope_yellow >= 1 or slope_yellow == 999:
                            angle, distance1, x_coor, depth = mc.immediate_motion_needed_2(yellow_cone_connect_sequence[0], yellow_cone_connect_sequence[1]) # -2 for only yellow cone detected
                        else:
                            angle, distance1, x_coor, depth = mc.immediate_motion_needed(yellow_cone_connect_sequence[0], -2)
            elif red_cone_connect_sequence:
                    angle, distance1, x_coor, depth = mc.immediate_motion_needed(red_cone_connect_sequence[0], 2)
                    
            # servo_adjust = 0
            # speed = 100
    '''
    if go_to_second_layer == 2:
        #second layer
        
        if apex_coor:
            apex_on_map(top_view_map, apex_coor, side)
            angle, distance1, x_coor, depth, next_point, distance2 = mc.motion_needed(apex_coor, side, yellow_cone_connect_sequence, red_cone_connect_sequence)
        
            if len(yccs_tv) == 3 and len(rccs_tv):
                cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), path_pt_0, (0,0,200), 1)
                cv2.arrowedLine(top_view_map, path_pt_0, path_pt_1, (0,0,200), 1)
                cv2.arrowedLine(top_view_map, path_pt_1, path_pt_2, (0,0,200), 1)

        else: #straigh
            # cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), path_pt_0, (10,255,10), 1)
            color1 = distance_2_color(path_pt_0, path_pt_1)
            color2 = distance_2_color(path_pt_1, path_pt_2)
            cv2.arrowedLine(top_view_map, path_pt_0, path_pt_1, color1, 1)
            cv2.arrowedLine(top_view_map, path_pt_1, path_pt_2, color2, 1)
            
            first_cones_midpt = [0,0,[(yellow_cone_connect_sequence[0][2][0]+ red_cone_connect_sequence[0][2][0])/2, 0 , (yellow_cone_connect_sequence[0][2][2]+ red_cone_connect_sequence[0][2][2])/2]]
            angle, distance1, x_coor, depth = mc.immediate_motion_needed(first_cones_midpt, 0)
            print("Move Straight!")
            # servo_adjust = 0
            # speed = 100
    '''
    if go_to_second_layer == 2: #both ccs detected
            side = 0
            apex_coor, side = acc.simple_apex_detect(image, yellow_cone_connect_sequence, red_cone_connect_sequence)
            if apex_coor: #if apex detected
                    print("apex: ", apex_coor)
                    apex_on_map(top_view_map, apex_coor, side)
                    
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
                    print("Move Straight!")
                    # if yccs_tv and rccs_tv:
                    color1 = maptv.distance_2_color(path_pt_0, path_pt_1)
                    color2 = maptv.distance_2_color(path_pt_1, path_pt_2)
                    cv2.arrowedLine(top_view_map, path_pt_0, path_pt_1, color1, 1)
                    cv2.arrowedLine(top_view_map, path_pt_1, path_pt_2, color2, 1)
                    
                    first_cones_midpt = [0,0,[(yellow_cone_connect_sequence[0][2][0]+ red_cone_connect_sequence[0][2][0])/2, 0 , (yellow_cone_connect_sequence[0][2][2]+ red_cone_connect_sequence[0][2][2])/2]]
                    angle, distance1, x_coor, depth = mc.immediate_motion_needed(first_cones_midpt, 0)
                                            
        # angle_raw = round(angle,1)
        # distance1_raw = round(distance1,1)
        # distance2_raw = round(distance2,1)
                
    
    angle = round(angle,1)
    distance1 = round(distance1,1)
    distance2 = round(distance2,1)

    if depth != 0:
        color = distance_2_color((bg_mid_x, starting_height), (int(bg_mid_x + x_coor), int(starting_height - depth)))
        cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), (int(bg_mid_x + x_coor), int(starting_height - depth)), color, 1)
    if next_point:
        cv2.putText(top_view_map, "Exit PT",(bg_mid_x + next_point[0] - 30, starting_height - next_point[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,10), 1)
        color_np = distance_2_color((int(bg_mid_x + x_coor), int(starting_height - depth)), (bg_mid_x + next_point[0], starting_height - next_point[1]))
        cv2.arrowedLine(top_view_map, (int(bg_mid_x + x_coor), int(starting_height - depth)), (bg_mid_x + next_point[0], starting_height - next_point[1])  , color_np, 1)
    # servo_adjust = round(mc.steering(angle))
    
    # if servo_adjust >= 10:
    #     speed = round(mc.speed_control(distance1))
    # else:
    #     speed = 100

    if angle != 999 and speed != -999:
            servo_adjust = round(mc.steering(angle))
            servo_adjust_round_10 = round(mc.steering(angle)/10)*10
            speed = round(mc.speed_control(distance1))
    elif angle == 999 and speed == -999:
            servo_adjust = 0
            servo_adjust_round_10 = 0
            speed = 0

    print("angular distance: ", angle)
    print("distance1: ", distance1)
    print("distance2: ", distance2)
    print("servo_adjust: ", servo_adjust)
    print("speed: ", speed)
    
        
    # elif side == 0:
    #     cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), (bg_mid_x, int(starting_height - 50)), (10,255,10), 1)
    
    if apex_coor:
        cv2.putText(top_view_map, "Angluar distance of the Apex: {}".format(angle), (5 ,height-55), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
        cv2.putText(top_view_map, "Distance of the apex: {}".format(distance1), (5 ,height-40), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
        cv2.putText(top_view_map, "Distance of exit pt.: {}".format(distance2), (5 ,height-70), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
    cv2.putText(top_view_map, "Servo Adjust: {}".format(servo_adjust), (5 ,height-25), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
    cv2.putText(top_view_map, "PWM: {}".format(speed), (5 ,height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)

    return top_view_map, servo_adjust, speed

while 1:
    #get trackbars values
    y1x = (cv2.getTrackbarPos("Y-1x", "Yellow Cones") - 100) / 100
    y2x = (cv2.getTrackbarPos("Y-2x", "Yellow Cones") - 100) / 100
    y3x = (cv2.getTrackbarPos("Y-3x", "Yellow Cones") - 100) / 100
    y1z = (cv2.getTrackbarPos("Y-1z", "Yellow Cones") ) / 100
    y2z = (cv2.getTrackbarPos("Y-2z", "Yellow Cones") ) / 100
    y3z = (cv2.getTrackbarPos("Y-3z", "Yellow Cones") ) / 100

    r1x = (cv2.getTrackbarPos("R-1x", "Red Cones") - 100) / 100
    r2x = (cv2.getTrackbarPos("R-2x", "Red Cones") - 100) / 100
    r3x = (cv2.getTrackbarPos("R-3x", "Red Cones") - 100) / 100
    r1z = (cv2.getTrackbarPos("R-1z", "Red Cones") ) / 100
    r2z = (cv2.getTrackbarPos("R-2z", "Red Cones") ) / 100
    r3z = (cv2.getTrackbarPos("R-3z", "Red Cones") ) / 100

    #right 2 no fps
    # center_with_depth_yellow = [[[93, 175], 64.3, [-0.2825795114040375, 0.0559876412153244, 0.6420000195503235]], [[138, 142], 126.3, [-0.3470691442489624, 0.023156538605690002, 1.25600004196167]], [[214, 132], 147.3, [-0.01610095053911209, -0.04037623479962349, 1.4730000495910645]]]
    # center_with_depth_red = [[[377, 170], 62.3, [0.3294704854488373, 0.06442820280790329, 0.6220000386238098]], [[299, 141], 112.6, [0.29812708497047424, -0.023531462997198105, 1.128000020980835]], [[339, 126], 146.7, [0.5794180631637573, 0.10597564280033112, 1.4300000667572021]]]
    
    # yellow_cone_connect_sequence = [[[93, 175], 64.3, [-0.2825795114040375, 0.0559876412153244, 0.6420000195503235]], [[138, 142], 126.3, [-0.3470691442489624, 0.023156538605690002, 1.25600004196167]], [[214, 132], 147.3, [-0.01610095053911209, -0.04037623479962349, 1.4730000495910645]]]
    # red_cone_connect_sequence = [[[377, 170], 62.3, [0.3294704854488373, 0.06442820280790329, 0.6220000386238098]], [[299, 141], 112.6, [0.29812708497047424, -0.023531462997198105, 1.128000020980835]], [[339, 126], 146.7, [0.5794180631637573, 0.10597564280033112, 1.4300000667572021]]]
    # apex_coor = [[[299, 141], 112.6, [0.29812708497047424, -0.023531462997198105, 1.128000020980835]]]
    # side = 1

    #left 4 no fps
    # center_with_depth_yellow = [[[9, 201], 43.7, [-0.3090497553348541, 0.07514622807502747, 0.4360000193119049]], [[110, 182], 119.0, [-0.418509840965271, 0.18561571836471558, 1.190000057220459]], [[62, 172], 161.7, [-0.8123952746391296, 0.22574199736118317, 1.6170001029968262]]]
    # center_with_depth_red = [[[407, 196], 71.8, [0.462015837430954, 0.1423620581626892, 0.7170000076293945]], [[320, 176], 128.9, [0.42703935503959656, 0.21675916016101837, 1.2820000648498535]], [[231, 169], 183.1, [0.08197413384914398, 0.26761001348495483, 1.8310000896453857]]]

    # yellow_cone_connect_sequence = [[[9, 201], 43.7, [-0.3090497553348541, 0.07514622807502747, 0.4360000193119049]], [[110, 182], 119.0, [-0.418509840965271, 0.18561571836471558, 1.190000057220459]], [[62, 172], 161.7, [-0.8123952746391296, 0.22574199736118317, 1.6170001029968262]]]
    # red_cone_connect_sequence = [[[407, 196], 71.8, [0.462015837430954, 0.1423620581626892, 0.7170000076293945]], [[320, 176], 128.9, [0.42703935503959656, 0.21675916016101837, 1.2820000648498535]], [[231, 169], 183.1, [0.08197413384914398, 0.26761001348495483, 1.8310000896453857]]]
    # apex_coor = [[[110, 182], 119.0, [-0.418509840965271, 0.18561571836471558, 1.190000057220459]]]
    # side = -1

    #straight 2 no fps
    # center_with_depth_yellow = [[[112, 175], 63.5, [-0.245433509349823, 0.06556760519742966, 0.6330000162124634]], [[170, 142], 120.0, [-0.22113445401191711, -0.017145255580544472, 1.1980000734329224]], [[189, 130], 178.3, [-0.1767275184392929, -0.05455920472741127, 1.7780001163482666]]]
    # center_with_depth_red = [[[363, 171], 61.2, [0.2920898199081421, 0.06539657711982727, 0.612000048160553]], [[303, 137], 117.9, [0.32319527864456177, -0.01687333546578884, 1.1790000200271606]], [[283, 125], 173.7, [0.390788197517395, 0.0035827087704092264, 1.7370001077651978]]]
    
    # yellow_cone_connect_sequence = [[[112, 175], 63.5, [-0.245433509349823, 0.06556760519742966, 0.6330000162124634]], [[170, 142], 120.0, [-0.22113445401191711, -0.017145255580544472, 1.1980000734329224]], [[189, 130], 178.3, [-0.1767275184392929, -0.05455920472741127, 1.7780001163482666]]]
    # red_cone_connect_sequence = [[[363, 171], 61.2, [0.2920898199081421, 0.06539657711982727, 0.612000048160553]], [[303, 137], 117.9, [0.32319527864456177, -0.01687333546578884, 1.1790000200271606]], [[283, 125], 173.7, [0.390788197517395, 0.0035827087704092264, 1.7370001077651978]]]
    # apex_coor = []
    # side = 0

    #straight 3 no fps
    # center_with_depth_yellow =  [[[9, 200], 43.6, [-0.3090497553348541, 0.0722905769944191, 0.4360000193119049]], [[111, 182], 118.3, [-0.40168479084968567, 0.1794281005859375, 1.1750000715255737]], [[150, 169], 181.6, [-0.41142502427101135, 0.20537972450256348, 1.811000108718872]]]
    # center_with_depth_red = [[[406, 193], 71.4, [0.46710100769996643, 0.13475172221660614, 0.7140000462532043]], [[320, 177], 128.6, [0.43861204385757446, 0.22044043242931366, 1.2790000438690186]], [[289, 167], 182.6, [0.4119362235069275, 0.23762892186641693, 1.8310000896453857]]]

    # yellow_cone_connect_sequence =  [[[9, 200], 43.6, [-0.3090497553348541, 0.0722905769944191, 0.4360000193119049]], [[111, 182], 118.3, [-0.40168479084968567, 0.1794281005859375, 1.1750000715255737]], [[150, 169], 181.6, [-0.41142502427101135, 0.20537972450256348, 1.811000108718872]]]
    # red_cone_connect_sequence = [[[406, 193], 71.4, [0.46710100769996643, 0.13475172221660614, 0.7140000462532043]], [[320, 177], 128.6, [0.43861204385757446, 0.22044043242931366, 1.2790000438690186]], [[289, 167], 182.6, [0.4119362235069275, 0.23762892186641693, 1.8310000896453857]]]
    # apex_coor = []
    # side = 0

    #none
    # yellow_cone_connect_sequence = []
    # red_cone_connect_sequence = []
    # apex_coor = []
    # side = 0

    #90 turn left
    # center_with_depth_yellow =  [[[9, 200], 43.6, [-0.4090497553348541, 0.0722905769944191, 0.4360000193119049]], [[111, 182], 118.3, [-0.20168479084968567, 0.1794281005859375, 0.4378]], [[150, 169], 181.6, [-0.41142502427101135, 0.20537972450256348, 1.811000108718872]]]
    # center_with_depth_red = [[[406, 193], 71.4, [0.46710100769996643, 0.13475172221660614, 0.7140000462532043]], [[320, 177], 128.6, [0.43861204385757446, 0.22044043242931366, 0.7]], [[289, 167], 182.6, [0.4119362235069275, 0.23762892186641693, 1.8310000896453857]]]

    # yellow_cone_connect_sequence =  [[[9, 200], 43.6, [-0.3090497553348541, 0.0722905769944191, 0.4360000193119049]], [[111, 182], 118.3, [-0.40168479084968567, 0.1794281005859375, 1.1750000715255737]], [[150, 169], 181.6, [-0.41142502427101135, 0.20537972450256348, 1.811000108718872]]]
    # red_cone_connect_sequence = [[[406, 193], 71.4, [0.46710100769996643, 0.13475172221660614, 0.7140000462532043]], [[320, 177], 128.6, [0.43861204385757446, 0.22044043242931366, 1.2790000438690186]], [[289, 167], 182.6, [0.4119362235069275, 0.23762892186641693, 1.8310000896453857]]]
    # apex_coor = []
    # side = 0

    #right 3 90d
    # center_with_depth_yellow = [[[92, 186], 70.8, [-0.31858906149864197, 0.13593792915344238, 0.7080000042915344]], [[171, 136], 201.0, [-0.3108157217502594, 0.08288653194904327, 2.004000186920166]], [[265, 136], 196.9, [0.28083813190460205, 0.1519041508436203, 1.9630000591278076]]]
    # center_with_depth_red =  [[[344, 182], 72.0, [0.3176850378513336, 0.07693714648485184, 0.7200000286102295]], [[286, 150], 137.5, [0.30033546686172485, 0.08839099854230881, 1.3750001192092896]], [[391, 150], 136.4, [0.7627269625663757, 0.047482047230005264, 1.3640000820159912]]]
    # yellow_cone_connect_sequence = center_with_depth_yellow
    # red_cone_connect_sequence = center_with_depth_red

    #left 3 90d
    # center_with_depth_yellow = [[[93, 186], 70.9, [-0.3008784055709839, 0.12004602700471878, 0.7100000381469727]], [[150, 151], 134.0, [-0.3088132441043854, 0.09491758048534393, 1.340000033378601]], [[39, 151], 135.9, [-0.7967835068702698, 0.05608489736914635, 1.3560000658035278]]]
    # center_with_depth_red = [[[344, 183], 72.0, [0.3140120804309845, 0.07426851242780685, 0.7170000076293945]], [[263, 137], 201.0, [0.28670385479927063, 0.08944930136203766, 2.004000186920166]], [[171, 137], 199.8, [-0.31643161177635193, 0.049922820180654526, 1.998000144958496]]]
    # yellow_cone_connect_sequence = center_with_depth_yellow
    # red_cone_connect_sequence = center_with_depth_red

    image = background
    
    #variable cones
    center_with_depth_yellow = [[[0,0], y1z*100, [y1x,0,y1z]], [[0,0], y2z*100, [y2x,0,y2z]]]#, [[0,0], y3z*100, [y3x,0,y3z]]]
    center_with_depth_red = []#[[[0,0], r1z*100, [r1x,0,r1z]], [[0,0], r2z*100, [r2x,0,r2z]], [[0,0], r3z*100, [r3x,0,r3z]]]

    yellow_cone_connect_sequence = center_with_depth_yellow
    red_cone_connect_sequence = center_with_depth_red
    
    side, directions = acc.simple_side_detect(yellow_cone_connect_sequence, red_cone_connect_sequence)
    print("side: ", side)
    print("direction: ", directions)
    apex_coor, _ = acc.simple_apex_detect(image, yellow_cone_connect_sequence, red_cone_connect_sequence)
    # apex_coor, side = acc.apex_detect(yellow_cone_connect_sequence,red_cone_connect_sequence)

    top_view_map, servo_adjust, speed = instance_map(center_with_depth_yellow, center_with_depth_red, yellow_cone_connect_sequence, red_cone_connect_sequence, apex_coor, side)
    servo_sign = np.sign(servo_adjust)
    print("servo_sign: ",servo_sign)
    sign = 0
    if servo_sign == 1:
        sign = "L"
    elif servo_sign == -1:
        sign = "R"
    elif servo_sign == 0:
        sign = "S"
    print("sign: ", sign)
    
    servo_adjust_for_send = "0"
    if 0 <= abs(servo_adjust) < 10: #single digit
        servo_adjust_for_send = "0{}".format(abs(servo_adjust))
    else: #double digits
        servo_adjust_for_send = "{}".format(abs(servo_adjust))
    
    speed_for_send = "0"
    if 0 <= speed < 10: #single digit
        speed_for_send = "00{}".format(speed)
    elif 10 <= speed < 100: #double digits
        speed_for_send = "0{}".format(speed)
    elif speed == 100: #100
        speed_for_send = "{}".format(speed)

    # message = "{}{}".format(sign, servo_adjust_for_send)
    message = "F{}{}{}".format(speed_for_send, sign, servo_adjust_for_send)
    print("message: ", message)
    cv2.putText(top_view_map, "Message: {}".format(message), (bg_mid_x + 50 ,height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
    # ser.write(message.encode())
    cv2.imshow("map", top_view_map)

    #capture results
    time_for_capture = 0
    k = cv2.waitKey(33)
    if k == ord('a'): #press a to cap
        time_for_capture = 1
        if time_for_capture == 1:
            time_for_capture = 0
            path_dir = "/Volumes/HenrySSD/Polyu_EIE/FYP/14APR/result_cap_image/with_tv_changed_message/right/3 90d/"
            cv2.imwrite(os.path.join(path_dir, "top-view_map_fix.jpg"), top_view_map)

    key = cv2.waitKey(25)
    if key == 27:
        break

cv2.destroyAllWindows()
