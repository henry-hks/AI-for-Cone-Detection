import cv2
import numpy as np 
import fuzzy

def steering_control(apex_coor):
  #fuzzy steering inferencing based on real x coordinates
  
  #servo clockwise and counterclockwise ratio
  servo_adjust = 0

  #get the apex's real x coordinates
  x_coor = apex_coor[0][2][0]

  #servo pitch max & min
  pitch_min = -15
  pitch_max = 15

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
    
def speed_control(apex_coor):
  #fuzzy speed inferencing based on distance of the apex
  
  #speed add or minus (PWM)
  speed_adjust = 0

  #get the distance of the apex (cm)
  apex_distance = apex_coor[0][1]

  #motor PWM adjusting max & min (6V: 120cm/s)
  pwm_min = 0
  pwm_max = 100
  
  # 1. fuzzification
  #     1. VC : Very Close
  #     2. C  : Close
  #     3. O  : Optimal
  #     4. F  : Far
  #     5. VF : Very Far

  # 2. inferencing

  #inferencing VC & C
  w1 = max(fuzzy.rmf(apex_distance, 60, 100), fuzzy.trimf(apex_distance, 60, 100, 120))
  # *think below 60cm case*
  #inferencing C & O
  w2 = max(fuzzy.trimf(apex_distance, 60, 100, 120), fuzzy.trimf(apex_distance, 100, 120, 140))

  #inferencing O & F
  w3 = max(fuzzy.trimf(apex_distance, 100, 120, 140), fuzzy.trimf(apex_distance, 120, 140, 180))
  
  #inferencing F & VF
  w4 = max(fuzzy.trimf(apex_distance, 120, 140, 180), fuzzy.lmf(apex_distance, 140, 180))

  i = max(w1, w2) #close or optimal
  j = max(w3, w4) #optimal or far

  speed_adjust = (i*pwm_min + j*pwm_max) #find the speed (pwm) for approaching the apex

  return speed_adjust