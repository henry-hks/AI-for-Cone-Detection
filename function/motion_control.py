import cv2
import numpy as np 
import fuzzy
import math

def slopes_sides_fuzzy(avg_of_avg_slopes, yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
  #fuzzy sides based on avg_of_avg_slopes
  shift = 0

  path_pt_2 = (int((yellow_cone_connect_sequence_tv[2][0]+ red_cone_connect_sequence_tv[2][0])/2) , int((yellow_cone_connect_sequence_tv[2][1]+ red_cone_connect_sequence_tv[2][1])/2))


  side_min = int(path_pt_2[0] - yellow_cone_connect_sequence_tv[2][0] + 10)
  side_max = int(red_cone_connect_sequence_tv[2][0] - path_pt_2[0] - 10)

  #inferencing VL & L
  w1 = max(fuzzy.rmf(avg_of_avg_slopes, -40, -35), fuzzy.trapmf(avg_of_avg_slopes, -40, -35, -20, -10))

  #inferencing L & S
  w2 = max(fuzzy.trapmf(avg_of_avg_slopes, -40, -35, -20, 0), fuzzy.trimf(avg_of_avg_slopes, -5, 0, 5))

  #inferencing S & R
  w3 = max(fuzzy.trimf(avg_of_avg_slopes, -5, 0, 5), fuzzy.trapmf(avg_of_avg_slopes, 0, 20, 35, 40))
  
  #inferencing R & VR
  w4 = max(fuzzy.trapmf(avg_of_avg_slopes, 10, 20, 35, 40), fuzzy.lmf(avg_of_avg_slopes, 35, 40))

  i = max(w1, w2) #left or straight
  j = max(w3, w4) #straight or right

  servo_adjust = (i*pitch_max + j*pitch_min) #find the ratio of clockwise and counterclockwise

  return servo_adjust



def motion_for_no_apex(yellow_cone_connect_sequence_tv, red_cone_connect_sequence_tv):
  # yellow_slope_1 = yellow_cone_connect_sequence_tv[]
  d =1 

def immediate_motion_needed(cone_center, side):
  shift_in = 0.1
  shift_to_mid = 0.3 #shift to the predicted mid
  mid_point_add_depth = 0.1
  distance = 0
  angle = 0
  x_coor = 0

  if side == -1: #left
    x_coor = cone_center[2][0] + shift_in
    depth = cone_center[2][2]
  elif side == 1: #right
    x_coor = cone_center[2][0] - shift_in
    depth = cone_center[2][2]
  elif side == 0: #both detected; go for midpoint of first pair of cones
    x_coor = cone_center[2][0]
    depth = cone_center[2][2] #+ mid_point_add_depth #add depth for the midpoint
  elif side == -2: #only yellow detected; shift to the predicted mid
    x_coor = cone_center[2][0] + shift_to_mid
    depth = cone_center[2][2] #+ mid_point_add_depth #add depth for the midpoint
  elif side == 2: #only red detected; shift to the predicted mid
    x_coor = cone_center[2][0] - shift_to_mid
    depth = cone_center[2][2] #+ mid_point_add_depth #add depth for the midpoint
  
  print("depth: ",depth)
  print("x_coor: ", x_coor)
  if depth != 0:
    angle_rad = math.atan(x_coor/depth)
    angle = angle_rad * 180 / math.pi
    distance = math.sqrt(x_coor**2 + depth**2)*100

  return angle, distance, x_coor*100 , depth*100

def motion_needed(apex_coor, side, yellow_cone_connect_sequence, red_cone_connect_sequence):
  shift_in = 0.1
  distance1 = 0
  distance2 = 0
  angle = 0
  x_coor = 0
  next_point = ()
  
  if side == -1: #left
    x_coor = apex_coor[0][2][0] + shift_in
  elif side == 1: #right
    x_coor = apex_coor[0][2][0] - shift_in
  
  depth = apex_coor[0][2][2]
  
  if side != 0 and depth != 0:
    
    angle_rad = math.atan(x_coor/depth)
    angle = angle_rad * 180 / math.pi
    distance1 = math.sqrt(x_coor**2 + depth**2)*100

    next_point_x = math.tan(angle_rad) * 0.5*(yellow_cone_connect_sequence[2][2][2] + red_cone_connect_sequence[2][2][2]) * 100
    next_point_y = 0.5*(yellow_cone_connect_sequence[2][2][2] + red_cone_connect_sequence[2][2][2]) * 100

    if next_point_x < (yellow_cone_connect_sequence[2][2][0] * 100) + 10:
      next_point_x = yellow_cone_connect_sequence[2][2][0] * 100 + 10
    elif next_point_x > (red_cone_connect_sequence[2][2][0] * 100) - 10:
      next_point_x = red_cone_connect_sequence[2][2][0] * 100 - 10

    # if side == -1: #left
    #   next_point_x = math.tan(angle_rad) * yellow_cone_connect_sequence[2][2][2] * 100
    #   next_point_y = yellow_cone_connect_sequence[2][2][2] * 100
    #   if next_point_x < (yellow_cone_connect_sequence[2][2][0] * 100) + 100:
    #     next_point_x = yellow_cone_connect_sequence[2][2][0] * 100 + 100
    # elif side == 1: #right
    #   next_point_x = math.tan(angle_rad) * red_cone_connect_sequence[2][2][2] * 100
    #   next_point_y = red_cone_connect_sequence[2][2][2] * 100
    #   if next_point_x > (red_cone_connect_sequence[2][2][0] * 100) - 100:
    #     next_point_x = red_cone_connect_sequence[2][2][0] * 100 - 100
    
    next_point = (int(next_point_x), int(next_point_y))
    if next_point:
      distance2 = math.sqrt(next_point[0]**2 + next_point[1]**2)

  return angle, distance1, x_coor*100, depth*100, next_point, distance2

def steering(angle):
  #fuzzy steering bases on angle needed
  servo_adjust = 0

  pitch_min = -40
  pitch_max = 40

  #inferencing VL & L
  w1 = max(fuzzy.rmf(angle, -40, -35), fuzzy.trapmf(angle, -40, -35, -20, -10))

  #inferencing L & S
  w2 = max(fuzzy.trapmf(angle, -40, -35, -20, 0), fuzzy.trimf(angle, -5, 0, 5))

  #inferencing S & R
  w3 = max(fuzzy.trimf(angle, -5, 0, 5), fuzzy.trapmf(angle, 0, 20, 35, 40))
  
  #inferencing R & VR
  w4 = max(fuzzy.trapmf(angle, 10, 20, 35, 40), fuzzy.lmf(angle, 35, 40))

  i = max(w1, w2) #left or straight
  j = max(w3, w4) #straight or right

  servo_adjust = (i*pitch_max + j*pitch_min) #find the ratio of clockwise and counterclockwise

  return servo_adjust
  

def steering_control(apex_coor, side):
  #fuzzy steering inferencing based on real x coordinates
  
  #servo clockwise and counterclockwise ratio
  servo_adjust = 0

  #get the apex's real x coordinates
  x_coor = apex_coor[0][2][0]

  #servo pitch max & min
  pitch_min = -15
  pitch_max = 15

  #the car need to turn less to avoid crashing into the apex
    #if it's left side, car needs to add a value of real x coordinates
    #if it's right side, car needs to minus a value of real x coordinates
  shift_value = 0.1 #10cm
  if side == -1: #left side
    x_coor = x_coor - shift_value
  elif side == 1:
    x_coor = x_coor + shift_value
 
  # 1. fuzzification
  #     1. VL : Very Left
  #     2. L  : Left
  #     3. S  : Straight
  #     4. R  : Right
  #     5. VR : Very Right

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
    
def speed_control(distance):
  #fuzzy speed inferencing based on distance of the apex
  
  #speed add or minus (PWM)
  speed_adjust = 0

  #get the distance of the apex (cm)
  # apex_distance = apex_coor[0][1]
  apex_distance = distance

  #motor PWM adjusting max & min (6V: 120cm/s)
  pwm_min = 0
  pwm_max = 40
  
  # 1. fuzzification
  #     1. VC : Very Close
  #     2. C  : Close
  #     3. O  : Optimal
  #     4. F  : Far
  #     5. VF : Very Far

  # 2. inferencing

  #inferencing VC & C
  w1 = max(fuzzy.rmf(apex_distance, 40, 60), fuzzy.trimf(apex_distance, 40, 60, 100))
  # *think below 60cm case*
  #inferencing C & O
  w2 = max(fuzzy.trimf(apex_distance, 40, 60, 100), fuzzy.trimf(apex_distance, 60, 100, 140))

  #inferencing O & F
  w3 = max(fuzzy.trimf(apex_distance, 60, 100, 140), fuzzy.trimf(apex_distance, 120, 140, 180))
  
  #inferencing F & VF
  w4 = max(fuzzy.trimf(apex_distance, 120, 140, 180), fuzzy.lmf(apex_distance, 140, 180))

  i = max(w1, w2) #close or optimal
  j = max(w3, w4) #optimal or far

  speed_adjust = (i*pwm_min + j*pwm_max) #find the speed (pwm) for approaching the apex
  if speed_adjust >= 100:
    speed_adjust = 100

  return speed_adjust

# def median_filter_and_difference(list):
#   list
