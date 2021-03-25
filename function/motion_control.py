import cv2
import numpy as np 
import fuzzy

# def motion_control(apex_coor, side):
#   if apex_coor: #if have apex
#     if apex_coor[2][0] 

# def path_finding(yellow_cone_connect_sequence, red_cone_connect_sequence):
#   #based on 3D x-coordinates
#   if len(yellow_cone_connect_sequence) == 3 and (red_cone_connect_sequence) == 3:
#     yellow_x_coor_diff_array = get_differences_3d_xcoor(yellow_cone_connect_sequence)
#     red_x_coor_diff_array = get_differences_3d_xcoor(red_cone_connect_sequence)
    
#     diff_thres_1 = 0.2
#     diff_thres_2 = 

#     for i in range(2):
#       if yellow_x_coor_diff_array[i] < 0:
#         if red_x_coor_diff_array[i] > 0:
#           if -0.2 < yellow_cone_connect_sequence[i]:
            
#           if 2 >= yellow_x_coor_diff_array[i] >= 0.2 or 2 >= red_x_coor_diff_array[i] >= 0.2:
#             direction = [-1,-1]

#       elif yellow_x_coor_diff_array[i] > 0:
#         if red_x_coor_diff_array[i] > 0:
#           if 2 >= yellow_x_coor_diff_array[i] >= 0.2 or 2 >= red_x_coor_diff_array[i] >= 0.2:
#             direction = [-1,-1]

def steering(yolo_slopes_yellow):
  #fuzzy inferencing based on slopes
  #yellow
  slope1 = yolo_slopes_yellow[0]

  #inferencing S & L
  w1 = max(fuzzy.rmf(slope1 , -0.2, -0.45), fuzzy.trapmf(slope1, -0.45, -0.2, -0.05, -0.01))
  #inferencing L & VL
  w2 = max(fuzzy.trapmf(slope1, -0.45, -0.2, -0.05, -0.01), fuzzy.trimf(slope1, -0.05, -0.01, 0.01))
  #inferencing VL & VR
  w3 = max(fuzzy.trimf(slope1, -0.05, -0.01, 0.01), fuzzy.trimf(slope1, -0.01, 0.01, 0.05))
  #inferencing VR & R
  w4 = max(fuzzy.trimf(slope1, -0.01, 0.01, 0.05), fuzzy.trapmf(slope1, 0.01, 0.05))

  i = max(w1,w2)
  j = max(w3,w4)

  return i, j

def steering_control(apex_coor):
  #fuzzy inferencing based on real x coordinates
  
  #servo clockwise and counterclockwise ratio
  servo_adjust = 0

  #get the apex's real x coordinates
  x_coor = apex_coor[0][2][0]

  #servo pitch max & min
  pitch_min = -15
  pitch_max = 15

  # 1. fuzzification
  #     1. L : Left
  #     2. VL: Right
  #     3. R : Right
  #     4. VR: Very Right

  # 2. inferencing

  #inferencing VL & L
  w1 = max(fuzzy.rmf(x_coor, -0.45, -0.3), fuzzy.trapmf(x_coor, -0.45, -0.3, -0.2, 0))

  #inferencing L & S
  w2 = max(fuzzy.trapmf(x_coor,-0.45, -0.3, -0.2, 0), fuzzy.trimf(x_coor, -0.2, 0, 0.2))

  #inferencing S & R
  w3 = max(fuzzy.trimf(x_coor, -0.2, 0, 0.2), fuzzy.trapmf(x_coor, 0, 0.2, 0.3, 0.45))
  
  #inferencing R & VR
  w4 = max(fuzzy.trapmf(x_coor, 0, 0.2, 0.3, 0.45), fuzzy.lmf(x_coor, 0.3, 0.45))

  i = max(w1, w2) #left or straight
  j = max(w3, w4) #straight or right

  servo_adjust = (i*pitch_max + j*pitch_min) #find the ratio of clockwise and counterclockwise

  return servo_adjust
    
# def straight_control(cone_connect_sequence, color_id):
#   #color_id: 0 for yellow; 1 for red
#   if color_id == 0: #yellow cone
#     if <= cone_connect_sequence[0][2][0] <=

# def tune_left(cone_connect_sequence, color_id):
#   #color_id: 0 for yellow; 1 for red
#   if color_id == 0: #yellow cone
#     if cone_connect_sequence[0][2][0] >= 