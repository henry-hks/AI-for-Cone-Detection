import cv2
import numpy as np 
import math
import motion_control as mc

width = 300
height = 450
bg_mid_x = int(width/2)
starting_height = height - 100
background = np.zeros((height,width,3), np.uint8)

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
    fov_rad = 86 * math.pi/180
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
    if yellow_cone_connect_sequence and red_cone_connect_sequence:
        cv2.line(top_view_map, (yellow_cone_connect_sequence_tv[0][0], yellow_cone_connect_sequence_tv[0][1]) , (yellow_cone_connect_sequence_tv[1][0], yellow_cone_connect_sequence_tv[1][1]), (255,255,100), 1)
        cv2.line(top_view_map, (yellow_cone_connect_sequence_tv[1][0], yellow_cone_connect_sequence_tv[1][1]) , (yellow_cone_connect_sequence_tv[2][0], yellow_cone_connect_sequence_tv[2][1]), (255,255,100), 1)
        cv2.line(top_view_map, (red_cone_connect_sequence_tv[0][0], red_cone_connect_sequence_tv[0][1]) , (red_cone_connect_sequence_tv[1][0], red_cone_connect_sequence_tv[1][1]), (255,100,255), 1)
        cv2.line(top_view_map, (red_cone_connect_sequence_tv[1][0], red_cone_connect_sequence_tv[1][1]) , (red_cone_connect_sequence_tv[2][0], red_cone_connect_sequence_tv[2][1]), (255,100,255), 1)

def apex_on_map(top_view_map, apex_coor, side):
    width = 300
    height = 450
    bg_mid_x = int(width/2)
    starting_height = height - 100
    
    apex_coor_tv_x = int(bg_mid_x + 100*apex_coor[0][2][0])
    apex_coor_tv_y = int(starting_height - 100*apex_coor[0][2][2])
    cv2.drawMarker(top_view_map, (apex_coor_tv_x, apex_coor_tv_y), (0,200,200), cv2.MARKER_CROSS, 10,1,1)
    if side == -1:
        cv2.putText(top_view_map, "APEX", (apex_coor_tv_x - 50, apex_coor_tv_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,10), 1)
    elif side == 1:
        cv2.putText(top_view_map, "APEX", (apex_coor_tv_x + 10, apex_coor_tv_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,10), 1)

def get_mids(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
    #try to use cv2.ellipse
    # radius = 
    path_pt_0 = (int((yellow_cone_connect_sequence_tv[0][0]+ red_cone_connect_sequence_tv[0][0])/2) , int((yellow_cone_connect_sequence_tv[0][1]+ red_cone_connect_sequence_tv[0][1])/2))
    path_pt_1 = (int((yellow_cone_connect_sequence_tv[1][0]+ red_cone_connect_sequence_tv[1][0])/2) , int((yellow_cone_connect_sequence_tv[1][1]+ red_cone_connect_sequence_tv[1][1])/2))
    path_pt_2 = (int((yellow_cone_connect_sequence_tv[2][0]+ red_cone_connect_sequence_tv[2][0])/2) , int((yellow_cone_connect_sequence_tv[2][1]+ red_cone_connect_sequence_tv[2][1])/2))

    return path_pt_0, path_pt_1, path_pt_2
        
# def path_predict(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv, servo_adjust, distance):

def get_better_path(top_view_map, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
    path_pt_0 = (int((yellow_cone_connect_sequence_tv[0][0]+ red_cone_connect_sequence_tv[0][0])/2) , int((yellow_cone_connect_sequence_tv[0][1]+ red_cone_connect_sequence_tv[0][1])/2))
    path_pt_1 = (int((yellow_cone_connect_sequence_tv[1][0]+ red_cone_connect_sequence_tv[1][0])/2) , int((yellow_cone_connect_sequence_tv[1][1]+ red_cone_connect_sequence_tv[1][1])/2))
    path_pt_2 = (int((yellow_cone_connect_sequence_tv[2][0]+ red_cone_connect_sequence_tv[2][0])/2) , int((yellow_cone_connect_sequence_tv[2][1]+ red_cone_connect_sequence_tv[2][1])/2))
    
    pt_difference_1 = abs(path_pt_0[0] - path_pt_1[0])
    pt_difference_2 = abs(path_pt_1[0] - path_pt_2[0])
    print("pt difference 1: ", pt_difference_1)
    print("pt difference 2: ", pt_difference_2)

    # if pt_difference_1 - pt_difference_2 <= 15:


def instance_map(center_with_depth_yellow, center_with_depth_red, yellow_cone_connect_sequence, red_cone_connect_sequence, apex_coor, side):
    width = 300
    height = 450
    bg_mid_x = int(width/2)
    starting_height = height - 100
    go_to_second_layer = 0
    
    #initial values
    angle = 0
    distance = 0
    servo_adjust = 0
    speed = 0
    
    top_view_map, yccs_tv, rccs_tv = get_map(yellow_cone_connect_sequence, red_cone_connect_sequence)
    connect_on_map(top_view_map, yccs_tv, rccs_tv)
    path_pt_0, path_pt_1, path_pt_2 = get_mids(top_view_map, yccs_tv, rccs_tv)
    get_better_path(top_view_map, yccs_tv, rccs_tv)

    if not center_with_depth_yellow and not center_with_depth_red:
        print("No Cone Detected! Action: STOP / EXPLORE") #stop the car
        servo_adjust = 0
        speed = 0
    
    elif center_with_depth_yellow or center_with_depth_red:
        if center_with_depth_yellow:
            if center_with_depth_yellow[0][1] <= 60 and center_with_depth_yellow[0][2][0] >= -0.02: ##car crashing into the yellow cone
                print("Turn Right NOW!") # turn right immediately
                servo_adjust = -15
                speed = 5
            if yellow_cone_connect_sequence: #yellow cone detected
                # if yellow_cone_connect_sequence[0][1] <= 60 and yellow_cone_connect_sequence[0][2][0] >= -0.02: #car crashing into the yellow cone
                #         print("Turn Right Now!") #turn right immediately
                go_to_second_layer += 1
        
        if center_with_depth_red:
            if center_with_depth_red[0][1] <= 60 and center_with_depth_red[0][2][0] <= 0.02: ##car crashing into the red cone
                print("Turn left NOW!") # turn left immediately
                servo_adjust = 15
                speed = 5
            if red_cone_connect_sequence: #red cond detected
                # if red_cone_connect_sequence[0][1] <= 60 and red_cone_connect_sequence[0][2][0] <= 0.02: #car crashing into the red cone
                #         print("Turn Left Now!") #turn left immediately
                go_to_second_layer += 1
    
    # else:
    #     cv2.arrowedLine(top_view_map, (int(width/2), height-20), (int(width/2), int(height/2)), (10,255,10), 2)
    #     print("Move Straight!")
    #     servo_adjust = 0
    #     speed = 100

    if go_to_second_layer == 2:
        cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), path_pt_0, (0,0,200), 1)
        cv2.arrowedLine(top_view_map, path_pt_0, path_pt_1, (0,0,200), 1)
        cv2.arrowedLine(top_view_map, path_pt_1, path_pt_2, (0,0,200), 1)
        if apex_coor:
            apex_on_map(top_view_map, apex_coor, side)
            angle, distance, x_coor, depth = np.round(mc.motion_needed(apex_coor, side),3)
            angle = angle
            distance = distance
            cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), (int(bg_mid_x + x_coor), int(starting_height - depth)), (10,255,10), 1)
            
            servo_adjust = round(mc.steering(angle))
            speed = round(mc.speed_control(distance))
        
        else:
            # if side == 0:
                # cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), path_pt_0, (10,255,10), 1)
                # cv2.arrowedLine(top_view_map, path_pt_0, path_pt_1, (10,255,10), 1)
                # cv2.arrowedLine(top_view_map, path_pt_1, path_pt_2, (10,255,10), 1)
            print("Move Straight!")
            servo_adjust = 0
            speed = 100

    print("angular distance: ", angle)
    print("distance: ", distance)
    print("servo_adjust: ", servo_adjust)
    print("speed: ", speed)
        
    # elif side == 0:
    #     cv2.arrowedLine(top_view_map, (bg_mid_x, starting_height), (bg_mid_x, int(starting_height - 50)), (10,255,10), 1)
    
    cv2.putText(top_view_map, "Angluar distance of the apex: {}".format(angle), (5 ,height-55), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
    cv2.putText(top_view_map, "Distance of the apex: {}".format(distance), (5 ,height-40), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
    cv2.putText(top_view_map, "Servo adjust: {}".format(servo_adjust), (5 ,height-25), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)
    cv2.putText(top_view_map, "PWM: {}".format(speed), (5 ,height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1)

    return top_view_map

while 1:
    #right 2 no fps
    center_with_depth_yellow = [[[93, 175], 64.3, [-0.2825795114040375, 0.0559876412153244, 0.6420000195503235]], [[138, 142], 126.3, [-0.3470691442489624, 0.023156538605690002, 1.25600004196167]], [[214, 132], 147.3, [-0.01610095053911209, -0.04037623479962349, 1.4730000495910645]]]
    center_with_depth_red = [[[377, 170], 62.3, [0.3294704854488373, 0.06442820280790329, 0.6220000386238098]], [[299, 141], 112.6, [0.29812708497047424, -0.023531462997198105, 1.128000020980835]], [[339, 126], 146.7, [0.5794180631637573, 0.10597564280033112, 1.4300000667572021]]]
    
    yellow_cone_connect_sequence = [[[93, 175], 64.3, [-0.2825795114040375, 0.0559876412153244, 0.6420000195503235]], [[138, 142], 126.3, [-0.3470691442489624, 0.023156538605690002, 1.25600004196167]], [[214, 132], 147.3, [-0.01610095053911209, -0.04037623479962349, 1.4730000495910645]]]
    red_cone_connect_sequence = [[[377, 170], 62.3, [0.3294704854488373, 0.06442820280790329, 0.6220000386238098]], [[299, 141], 112.6, [0.29812708497047424, -0.023531462997198105, 1.128000020980835]], [[339, 126], 146.7, [0.5794180631637573, 0.10597564280033112, 1.4300000667572021]]]
    apex_coor = [[[299, 141], 112.6, [0.29812708497047424, -0.023531462997198105, 1.128000020980835]]]
    side = 1

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

    top_view_map = instance_map(center_with_depth_yellow, center_with_depth_red, yellow_cone_connect_sequence, red_cone_connect_sequence, apex_coor, side)

    cv2.imshow("map", top_view_map)
    key = cv2.waitKey(25)
    if key == 27:
        break

cv2.destroyAllWindows()