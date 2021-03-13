import cv2
import numpy as np 
import fuzzy



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

def steering_2(reference_point, center_with_depth):
    #fuzzy inferencing based on x coordinates
    
    #the x coordinates difference between the cone centers and reference point
    d0 = center_with_depth[0][0][0] - reference_point[0] 
    d1 = center_with_depth[1][0][0] - reference_point[0] 
    d2 = center_with_depth[2][0][0] - reference_point[0] 
    deviation = [d0, d1, d2]

def steering_3()