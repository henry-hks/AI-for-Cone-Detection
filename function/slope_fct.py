import cv2
import numpy as np 

def direction_detect(detected_both, yolo_yellow_cones_center, yolo_red_cones_center, yolo_slopes_yellow, yolo_slopes_red):
    direction = [0,0] # -2 = very left; -1 = left; 0 = straight; 1 = right; 2 = very right
    
    #very left
    thres_for_very_left_yellow = 0.01
    thres_for_very_left_red = 0.01

    #left
    thres_for_left_yellow = 0.1
    thres_for_left_red = 0.5

    #straight
    thres_for_straight_yellow = [-0.4, -0.2]
    thres_for_straight_red = [0.2, 0.4]

    #right
    thres_for_right_yellow = -0.5
    thres_for_right_red = -0.1

    #very right
    thres_for_very_left_yellow = -0.01
    thres_for_very_left_red = -0.01

    '''
    thres_array = []
    if yolo_slopes_red[0] > thres_for_straight_red[0] and yolo_slopes_red[1] > thres_for_straight_red[0] :  #straight or left turn
      if yolo_slopes_yellow[0] <= thres_for_straight_yellow[0] and yolo_slopes_yellow[1] <= thres_for_straight_yellow[0]:  #straight
        direction = 0
        if yolo_slopes_red[0] 
    '''
    
    yolo_slope_difference_yellow = yolo_slopes_yellow[0] - yolo_slopes_yellow[1]
    yolo_slope_difference_red= yolo_slopes_red[0] - yolo_slopes_red[1]
    if yolo_slope_difference_yellow <= 0.04: # gental slopes change 
      avg_slope_yellow = (yolo_slopes_yellow[0] + yolo_slopes_yellow[1])/2
      # straight
      if avg_slope_yellow > thres_for_straight_yellow[0] and avg_slope_yellow <= thres_for_straight_yellow[1]: 
        direction[0] = 0 
      # very left
      if yolo_slope_difference_red <= 0.04: # gental slopes change 
        avg_slope_red = (yolo_slopes_red[0] + yolo_slopes_red[1])/2
        if avg_slope_red > -thres_for_very_left_red or avg_slope_red <= thres_for_very_left_red:
          direction[0] = -2
      


def apex_detect(detected_both, yolo_yellow_cones_center, yolo_red_cones_center, yolo_slopes_yellow, yolo_slopes_red):
    yolo_slopes_checker_red = 0
    yolo_slopes_checker_yellow = 0
    if len(yolo_slopes_yellow) == 2:
      yolo_slope_difference_yellow = abs(yolo_slopes_yellow[0] - yolo_slopes_yellow[1])
      yolo_slopes_checker_yellow = 1
    else:
      yolo_slopes_checker_yellow = 0


    if len(yolo_slopes_red) == 2:
      yolo_slope_difference_red = abs(yolo_slopes_red[0] - yolo_slopes_red[1])
      yolo_slopes_checker_red = 1
    else:
      yolo_slopes_checker_red = 0
    
    if yolo_slopes_checker_red + yolo_slopes_checker_yellow == 2:
      print(yolo_slope_difference_yellow)
      if yolo_slope_difference_red >= 0.1 or yolo_slope_difference_yellow >= 0.1:
        print("sharp turn")
        if yolo_slope_difference_red >= 0.1:
          cx, cy = yolo_red_cones_center[1]
          cv2.drawMarker(detected_both, (cx, cy), (255,0,255), cv2.MARKER_TRIANGLE_UP, 10,1,1 )
          cv2.putText(detected_both, "APEX", (cx, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,100,255), 2)

        if yolo_slope_difference_yellow >= 0.1:
          cx, cy = yolo_yellow_cones_center[1]
          cv2.drawMarker(detected_both, (cx, cy), (255,0,255), cv2.MARKER_TRIANGLE_UP, 10,1,1 )
          cv2.putText(detected_both, "APEX", (cx, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,100,255), 2)
    
    return detected_both