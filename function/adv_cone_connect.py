import cv2
import numpy as np 
import fuzzy
import coordinates
import math
import sys
# sys.path.append("/home/fyp/darknet")
# import darknet as dn 
# import darknet_images as dni 

def bbox2points_(bbox):
  x, y, w, h = bbox
  xmin = int(round(x - (w / 2)))
  xmax = int(round(x + (w / 2)))
  ymin = int(round(y - (h / 2)))
  ymax = int(round(y + (h / 2)))
  return xmin, ymin, xmax, ymax

def getDepth_dcm(detections, depth_frame, depthimg, img, colors):
    center_with_depth = []
    for label, confidence, bbox in detections:
        x, y, w, h = bbox

        # print("bbox: ", bbox)
        x = int(round(x))
        y = int(round(y))
        w = int(round(w))
        h = int(round(h))

        cx = x
        # int(x+w/2)
        cy = y
        # int(y+h/2)
        # print("For dected {label} with confidence {confidence} , Depth at x={x} , y={y} is {depthimg[y][x]}".format(label=label, confidence=float(confidence),x=x, y=y, depthimg=depthimg))
        '''
        print("For detected ", label, "with confidence ", confidence, ", Depth at x= ", x , ", y= is", depthimg[int(y)][int(x)])
        '''
        # print("depth image of the boundary box is :")
        depth_bbox = depthimg[int(y-round(h/2)):int(y+round(h/2)),int(x-round(w/2)):int(x+round(w/2))]
        # print(depth_bbox)
        arr = np.array(depth_bbox)
        depth_median = np.median(arr)
        dm_d10 = np.round(depth_median/10,2)
        
        #cv2.putText(img,f'{depthimg[y][x]/10}cm', (x+round(w/2),y+round(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[label], 1)
        cv2.putText(img,"{} cm".format(dm_d10), (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,200,200), 1)
        '''
        print("The distance of the cone is ", dm_d10, "cm")
        '''
        # cv2.circle(img, (x,y), radius=1, color=(255,0,100), thickness=-1)
        cv2.drawMarker(img, (cx, cy), (255,200,200), cv2.MARKER_CROSS, 10,1,1 )

        # #cone localize
        xmin, ymin, xmax, ymax = bbox2points_(bbox)
        min = [10000,0,0]
        #x = math.floor(x)
        #y = math.floor(y)
        filtered_depth_img = cv2.GaussianBlur(depthimg, (3,3), 2, None, 2, cv2.BORDER_REPLICATE)
        depth_bboxf = filtered_depth_img[ymin:ymax,xmin:xmax]
        # print(depth_bbox)
        arrf = np.array(depth_bboxf)
        depth_medianf = np.median(arrf)

        for y_bbox in range(len(depth_bboxf)):
            for x_bbox in range(len(depth_bboxf[0])):
                abs_diff = abs(depth_medianf - depth_bboxf[y_bbox][x_bbox])
                if abs_diff < min[0]:
                    min = [abs_diff, y_bbox, x_bbox]

        if xmin <0 or ymin<0:
            xmin = 0
            ymin = 0
        coor = coordinates.get_real_world_coordinates(depth_frame, xmin + min[2], ymin + min[1])
        # if sum(coor) == 0.0:
        #   coor = []
        # coor = [1.4, 0.21, 5.2]
        
        # coor = coordinates.get_real_world_coordinates(depth_frame, x, y) 
        offset = 0.06
        coor[0] = coor[0] - offset
        if coor[2] != 0:
          center_with_depth.append([[x,y], dm_d10, coor])
        # center_with_depth.append([[cx,cy], dm_d10, coor])

    center_with_depth = sorted(center_with_depth, key=lambda k:[k[1], k[0][0]], reverse=False) #sort by depth and then 2D x coordinate ascendingly
    # print("center with depth", center_with_depth)
    return img, center_with_depth

def drawline_sequence(detected_both, cone_connect_sequence):
  cv2.line(detected_both, (cone_connect_sequence[0][0][0], cone_connect_sequence[0][0][1]) , (cone_connect_sequence[1][0][0], cone_connect_sequence[1][0][1]), (255,100,255), 2)
  cv2.line(detected_both, (cone_connect_sequence[1][0][0], cone_connect_sequence[1][0][1]) , (cone_connect_sequence[2][0][0], cone_connect_sequence[2][0][1]), (255,255,100), 2)

def get_cc_midpoint(cone_connect_sequence):
  mid_x_1 = int((cone_connect_sequence[0][0][0] + cone_connect_sequence[1][0][0]) / 2)
  mid_y_1 = int((cone_connect_sequence[0][0][1] + cone_connect_sequence[1][0][1]) / 2)
  mid_x_2 = int((cone_connect_sequence[1][0][0] + cone_connect_sequence[2][0][0]) / 2)
  mid_y_2 = int((cone_connect_sequence[1][0][1] + cone_connect_sequence[2][0][1]) / 2)

  return mid_x_1, mid_x_2, mid_y_1, mid_y_2

def find_borders_midpoints(yellow_cone_connect_sequence, red_cone_connect_sequence):
  mid_yellow = []
  mid_red = []
  
  x1_y, x2_y, y1_y, y2_y = get_cc_midpoint(yellow_cone_connect_sequence)
  x1_r, x2_r, y1_r, y2_r = get_cc_midpoint(red_cone_connect_sequence)

  mid_yellow.append([x1_y, y1_y], [x2_y, y2_y])
  mid_red.append([x1_r, y1_r], [x2_r, y2_r])

  return mid_yellow, mid_red

def cal_slopes(cone_connect_sequence):
  yolo_slopes = []
  for i in range(len(cone_connect_sequence)):
    if i != len(cone_connect_sequence)-1:
      #get the slopes between cones
      xdiff = (cone_connect_sequence[i][0][0]) - (cone_connect_sequence[i+1][0][0])
      ydiff = (cone_connect_sequence[i][0][1]) - (cone_connect_sequence[i+1][0][1])
      if xdiff != 0:
        sl = round(np.true_divide(ydiff, xdiff, out=None), 5)
      else:
        sl = 999
      yolo_slopes.append(sl)
  return yolo_slopes

def real_cal_slopes(cone_connect_sequence):
  yolo_slopes = []
  for i in range(len(cone_connect_sequence)):
    if i != len(cone_connect_sequence)-1:
      # #root depth
      # root_depth = cone_connect_sequence[i][2][2]

      # zdiff = 
      #get the slopes between yellow cones
      xdiff = (cone_connect_sequence[i][2][0]) - (cone_connect_sequence[i+1][2][0])
      ydiff = (cone_connect_sequence[i][2][2]) - (cone_connect_sequence[i+1][2][2])
      if ydiff != 0:
        sl = round(np.true_divide(xdiff, ydiff, out=None), 5)
      else:
        sl = 999
      yolo_slopes.append(sl)
  return yolo_slopes

def cone_connect_with_depth(detected_both, center_with_depth, color_id): #color_id: 0 for yellow, 1 for red
  cone_connect_sequence = []
  depth_diff_thres = 10
  depth_diff_u_thres = 200

  # if len(center_with_depth) == 2:
  #   if center_with_depth[0][1] != 0 and center_with_depth[1][1] != 0:
  #     depth_difference = center_with_depth[1][1] - center_with_depth[0][1] 
  #     if depth_diff_u_thres > depth_difference > depth_diff_thres:
  #       cone_connect_sequence=[center_with_depth[0], center_with_depth[1]]

  if len(center_with_depth) >= 3:
    if center_with_depth[0][1] != 0 and center_with_depth[1][1] != 0 and center_with_depth[2][1] != 0: #if no false depth
      depth_difference_1 = center_with_depth[1][1] - center_with_depth[0][1] 
      depth_difference_2 = center_with_depth[2][1] - center_with_depth[1][1]
      # print("depth diferences: ", depth_difference_1, depth_difference_2)
      if depth_diff_u_thres > depth_difference_1 > depth_diff_thres and depth_diff_u_thres > depth_difference_2 > depth_diff_thres: 
        #if cones are separated obviously and within the range of threshold
        cone_connect_sequence=[center_with_depth[0], center_with_depth[1], center_with_depth[2]]
        
      elif depth_difference_1 < depth_diff_thres and depth_diff_u_thres > depth_difference_2 > depth_diff_thres: #first two cones are horizontally aligned
        if center_with_depth[0][0][0] <= center_with_depth[2][0][0] <= center_with_depth[1][0][0]: 
          #if the further cone is horizontally between two near aligned cone pair
          if color_id == 0: # connect for yellow, probably sharp turn left
            cone_connect_sequence = [center_with_depth[1], center_with_depth[2], center_with_depth[0]]
            
          elif color_id == 1: #connect for red, probably sharp turn right
            cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]
        elif center_with_depth[0][0][0] >= center_with_depth[2][0][0] >= center_with_depth[1][0][0]:
          #if the further cone is horizontally between two near aligned cone pair
          if color_id == 0: # connect for yellow, probably sharp turn left
            cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]
            
          elif color_id == 1: #connect for red, probably sharp turn right
            cone_connect_sequence = [center_with_depth[1], center_with_depth[2], center_with_depth[0]]

        elif center_with_depth[2][0][0] <= center_with_depth[1][0][0] and center_with_depth[2][0][0] <= center_with_depth[0][0][0]:
          #connect for both, probably 90 degree turn left
          if center_with_depth[1][0][0] <= center_with_depth[0][0][0]:
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
          elif center_with_depth[1][0][0] >= center_with_depth[0][0][0]:
            cone_connect_sequence = [center_with_depth[1], center_with_depth[0], center_with_depth[2]]

        elif center_with_depth[2][0][0] >= center_with_depth[1][0][0] and center_with_depth[2][0][0] >= center_with_depth[0][0][0]:
          #connect for both, probably 90 degree turn right
          if center_with_depth[1][0][0] >= center_with_depth[0][0][0]:
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
          elif center_with_depth[1][0][0] <= center_with_depth[0][0][0]:
            cone_connect_sequence = [center_with_depth[1], center_with_depth[0], center_with_depth[2]]
      
      elif depth_difference_2 < depth_diff_thres and depth_diff_u_thres > depth_difference_1 > depth_diff_thres: #last two cones are horizontally aligned
        if center_with_depth[2][0][0] <= center_with_depth[0][0][0] <= center_with_depth[1][0][0]: 
          #if the further cone is horizontally between two near aligned cone pair
          if color_id == 0: # connect for yellow, probably sharp turn left
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
            
          elif color_id == 1: # connect for red, probably sharp turn left
            cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]

        elif center_with_depth[2][0][0] >= center_with_depth[0][0][0] >= center_with_depth[1][0][0]:
          if color_id == 0: # connect for yellow, probably sharp turn right
            cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]
            
          elif color_id == 1: # connect for red, probably sharp turn right
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
            
        elif center_with_depth[2][0][0] <= center_with_depth[0][0][0] and center_with_depth[1][0][0] <= center_with_depth[0][0][0]:
          #connect for both, turn left
          if center_with_depth[2][0][0] <= center_with_depth[1][0][0]:
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
          elif center_with_depth[2][0][0] >= center_with_depth[1][0][0]:
            cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]
          
        elif center_with_depth[2][0][0] >= center_with_depth[0][0][0] and center_with_depth[1][0][0] >= center_with_depth[0][0][0]:
          #connect for both, turn right
          if center_with_depth[2][0][0] >= center_with_depth[1][0][0]:
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
          elif center_with_depth[2][0][0] <= center_with_depth[1][0][0]:
            cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]
      
      elif depth_difference_1 < depth_diff_thres and depth_difference_2 < depth_diff_thres: #three cones are horizontally aligned
        if center_with_depth[0][0][0] <= center_with_depth[1][0][0] <= center_with_depth[2][0][0]:
          if color_id == 0: #yellow
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
            
          elif color_id == 1: #red
            cone_connect_sequence = [center_with_depth[2], center_with_depth[1], center_with_depth[0]]
            
        elif center_with_depth[2][0][0] <= center_with_depth[1][0][0] <= center_with_depth[0][0][0]:
          if color_id == 0: #yellow
            cone_connect_sequence = [center_with_depth[2], center_with_depth[1], center_with_depth[0]]
            
          elif color_id == 1: #red
            cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]

  # yolo_slopes=[]
  if len(cone_connect_sequence) == 3:
    #draw lines
    drawline_sequence(detected_both, cone_connect_sequence)
    #calculate slopes
    # yolo_slopes.append(cal_slopes(cone_connect_sequence))
    # yolo_slopes.append(real_cal_slopes(cone_connect_sequence))
  
  # elif len(cone_connect_sequence) == 2:
  #   cv2.line(detected_both, (cone_connect_sequence[0][0][0], cone_connect_sequence[0][0][1]) , (cone_connect_sequence[1][0][0], cone_connect_sequence[1][0][1]), (255,100,255), 2)

  return detected_both, cone_connect_sequence

def direction_detect(yolo_slopes_yellow, yolo_slopes_red):
  #based on 2D slopes
  direction = [0,0] # -2 = very left; -1 = left; 0 = straight; 1 = right; 2 = very right 

  #yellow
  #straight
  if yolo_slopes_yellow <= -0.5:
    direction[0]=0
  #left
  if -0.5 < yolo_slopes_yellow <= -0.01:
    direction[0]=-1
  #very left
  if -0.01 < yolo_slopes_yellow <= 0:
    direction[0]=-2
  #very right
  if 0 < yolo_slopes_yellow <=0.01:
    direction[0]=2
  #right
  if 0.01 < yolo_slopes_yellow:
    direction[0]=1

  #red
  #right
  if yolo_slopes_red <= -0.01:
    direction[1]=1
  #very right
  if -0.01 < yolo_slopes_red <= 0:
    direction[1]=2
  #very left
  if 0 < yolo_slopes_red <= 0.01:
    direction[1]=-2
  #left
  if 0.01 < yolo_slopes_red <= 0.5:
    direction[1]=-1
  #straight
  if 0.5 < yolo_slopes_red:
    direction[1]=0

  return direction
  
def real_direction_detect(yolo_slopes_yellow, yolo_slopes_red):
  #based on 3D slopes
  direction = [0,0] # -2 = very left; -1 = left; 0 = straight; 1 = right; 2 = very right 
  if (-0.1 >= yolo_slopes_yellow or yolo_slopes_yellow >= 0.1 or yolo_slopes_yellow == 999) or (-0.1 >= yolo_slopes_red or yolo_slopes_red >= 0.1 or yolo_slopes_red == 999):
    direction = [0,0] #straight
    print("straight")

  if (0.1 >= yolo_slopes_yellow) and (-0.1 <= yolo_slopes_red):
    direction = [-1,-1] #left
    print("left")
  if (-0.5 >= yolo_slopes_yellow) and (-0.5 >= yolo_slopes_red):
    direction = [1,1] #right
    print("right")
  
  if (0.5 <= yolo_slopes_yellow) and (0.5 <= yolo_slopes_red):
    direction = [-2,-2] #very left
    print("very left")
  if (-0.5 >= yolo_slopes_yellow) and (-0.5 >= yolo_slopes_red):
    direction = [2,2] #very right
    print("very right")
  
  return direction

def get_directions_array(yolo_slopes_yellow, yolo_slopes_red):
  directions = []
  if len(yolo_slopes_yellow) == 1 and len(yolo_slopes_red) == 1:
    if len(yolo_slopes_yellow[0]) == 2 and len(yolo_slopes_red[0]) == 2:
      for i in range(len(yolo_slopes_yellow[0])):
        # direction = direction_detect(yolo_slopes_yellow[0][i], yolo_slopes_red[0][i])
        direction = real_direction_detect(yolo_slopes_yellow[0][i], yolo_slopes_red[0][i])
        directions.append(direction)
  
  return directions

def apex_detect(detected_both, yellow_cone_connect_sequence, red_cone_connect_sequence, yolo_slopes_yellow, yolo_slopes_red, directions):
  apex = 0 # 0=no apex; 1=apex detected
  side = 0 # -1=left; 0=straight; 1=right
  apex_coor = []
  if len(directions) == 2:
    if (-2 <= directions[0][0] <= -1 and -2 <= directions[0][1] <= -1) and (-2 <= directions[1][0] <= -1 and -2 <= directions[1][1] <= -1):
      side = -1 #left
    if 1 <= directions[0][0] <= 2 and 1 <= directions[0][1] <= 2 and 1 <= directions[1][0] <= 2 and 1 <= directions[1][1] <= 2:
      side = 1 #right
    if directions[0][0] == 0 and directions[0][1] == 0 and directions[1][0] == 0 and directions[1][1] == 0:
      side = 0
  
  if side == -1:
    if len(yolo_slopes_yellow) == 2:
      slopes_difference = abs(yolo_slopes_yellow[0] - yolo_slopes_yellow[1])
      if slopes_difference >= 0.2:
        apex = 1
  
  if side == 1:
    if len(yolo_slopes_red) == 2:
      slopes_difference = abs(yolo_slopes_red[0] - yolo_slopes_red[1])
      if slopes_difference >= 0.2:
        apex = 1
  
  if apex == 1:
    if side == -1:
      apex_coor.append(yellow_cone_connect_sequence[1])
    if side == 1:
      apex_coor.append(red_cone_connect_sequence[1])

  print("apex: ", apex_coor)
  if apex_coor:
    cv2.drawMarker(detected_both, (apex_coor[0][0], apex_coor[0][1]), (0,200,200), cv2.MARKER_CROSS, 10,1,1 )
    cv2.putText(detected_both, "APEX", (apex_coor[0][0], apex_coor[0][1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8,(0,255,10), 1)
  
  return apex_coor

def get_differences_3d_xcoor(cone_connect_sequence):
  x_coor_diff_array = []
  if len(cone_connect_sequence) == 3:
    for i in range(3):
      if i != 2:
        x_coor_diff_1 = cone_connect_sequence[i][2][0] - cone_connect_sequence[i+1][2][0]
        x_coor_diff_array.append(x_coor_diff_1)

  return x_coor_diff_array

def simple_direction_detect(yellow_cone_connect_sequence, red_cone_connect_sequence):
  #based on 3D x-coordinates
  #based on top view slopes x_diff/y_diff
  direction = [0,0] #-1 = left; 0 = straight; 1 = right
  directions = [] #directions array
  thres_direction = 0.2
  yellow_slopes = real_cal_slopes(yellow_cone_connect_sequence)
  red_slopes = real_cal_slopes(red_cone_connect_sequence)
  if len(yellow_cone_connect_sequence) == 3 and len(red_cone_connect_sequence) == 3:
    # print("real coor yellow: ", yellow_cone_connect_sequence)
    # print("real coor red: ", red_cone_connect_sequence)
    # if yellow_cone_connect_sequence[0][2][2] != 0.0 and yellow_cone_connect_sequence[1][2][2] != 0.0 and yellow_cone_connect_sequence[2][2][2] != 0.0 and red_cone_connect_sequence[0][2][2] != 0.0 and red_cone_connect_sequence[1][2][2] != 0.0 and red_cone_connect_sequence[2][2][2] != 0.0:
    yellow_x_coor_diff_array = get_differences_3d_xcoor(yellow_cone_connect_sequence)
    red_x_coor_diff_array = get_differences_3d_xcoor(red_cone_connect_sequence)
    # print("yellow x coor difference: ", yellow_x_coor_diff_array)
    # print("red x coor difference: ", red_x_coor_diff_array)

    for i in range(2):
      if yellow_x_coor_diff_array[i] > thres_direction:
        direction[0] = -1
      if yellow_x_coor_diff_array[i] < -thres_direction:
        direction[0] = 1
      if red_x_coor_diff_array[i] > thres_direction:
        direction[1] = -1
      if red_x_coor_diff_array[i] < -thres_direction:
        direction[1] = 1
    
      directions.append(direction)
  
  if len(yellow_cone_connect_sequence) == 3 and len(red_cone_connect_sequence) != 3:
    print("only yellow side with 3 cones detected!")
    yellow_x_coor_diff_array = get_differences_3d_xcoor(yellow_cone_connect_sequence)
    for i in range(2):
      if yellow_x_coor_diff_array[i] > thres_direction:
        direction[0] = -1
        direction[1] = -1
      if yellow_x_coor_diff_array[i] < -thres_direction:
        direction[0] = 1
        direction[1] = 1

      directions.append(direction)

  if len(yellow_cone_connect_sequence) != 3 and len(red_cone_connect_sequence) == 3:
    print("only red side with 3 cones detected!")
    red_x_coor_diff_array = get_differences_3d_xcoor(red_cone_connect_sequence)
    for i in range(2):
      if red_x_coor_diff_array[i] > thres_direction:
        direction[1] = -1
        direction[0] = -1
      if red_x_coor_diff_array[i] < -thres_direction:
        direction[1] = 1
        direction[0] = 1

      directions.append(direction)

  return directions

def simple_side_detect(yellow_cone_connect_sequence, red_cone_connect_sequence):
  side = 999
  directions = simple_direction_detect(yellow_cone_connect_sequence,red_cone_connect_sequence)
  if directions:
    # if (directions[0].all() == -1) and (directions[1].all() == -1):
    if (directions[0][0] == -1 and directions[0][1] == -1) or (directions[1][0] == -1 and directions[1][1] == -1):
      side = -1 #left
      # cv2.putText(detected_both, "left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255), 2)
    # if (directions[0].all() == 1) and (directions[1].all() == 1):
    if (directions[0][0] == 1 and directions[0][1] == 1) or (directions[1][0] == 1 and directions[1][1] == 1):
      side = 1 #right
      # cv2.putText(detected_both, "right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255), 2)
    if directions[0][0] == 0 and directions[0][1] == 0 and directions[1][0] == 0 and directions[1][1] == 0:
      side = 0 #straight
      # cv2.putText(detected_both, "straight", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255), 2)
  
  return side, directions

def simple_apex_detect(detected_both, yellow_cone_connect_sequence, red_cone_connect_sequence):
  side = 999 # -1=left; 0=straight; 1=right
  apex_coor = []
  virtual_apex_coor = []
  tight_turn = 0
  thres_apex = 0.2
  
  side, directions = simple_side_detect(yellow_cone_connect_sequence, red_cone_connect_sequence)
  if side == -1:
    cv2.putText(detected_both, "left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255), 2)
  if side == 1:
    cv2.putText(detected_both, "right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255), 2)
  if side == 0:
    cv2.putText(detected_both, "straight", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255), 2)

  if directions:
    if len(yellow_cone_connect_sequence) == 3 and len(red_cone_connect_sequence) == 3:
      yellow_x_coor_diff_array = get_differences_3d_xcoor(yellow_cone_connect_sequence)
      red_x_coor_diff_array = get_differences_3d_xcoor(red_cone_connect_sequence)
      # print("yxcda: ", yellow_x_coor_diff_array)

      if abs(yellow_x_coor_diff_array[0] - yellow_x_coor_diff_array[1]) >= thres_apex or abs(red_x_coor_diff_array[0] - red_x_coor_diff_array[1]) >= thres_apex:
        if side == -1: #left
          apex_coor.append(yellow_cone_connect_sequence[1])
        if side == 1: #right
          apex_coor.append(red_cone_connect_sequence[1])

    if len(yellow_cone_connect_sequence) == 3 and len(red_cone_connect_sequence) != 3:
      if red_cone_connect_sequence:
        if side == 1: #right
          apex_coor.append(red_cone_connect_sequence[len(red_cone_connect_sequence)-1])
      else:
        if side == 1: #right
          tight_turn = 1 #tight turn right
          virtual_apex_coor.append([250, 170])
    
    if len(yellow_cone_connect_sequence) != 3 and len(red_cone_connect_sequence) == 3:
      if yellow_cone_connect_sequence:
        if side == -1: #left
          apex_coor.append(yellow_cone_connect_sequence[len(yellow_cone_connect_sequence)-1])
      else:
        if side == -1: #left
          tight_turn = -1 #tight turn left
          virtual_apex_coor.append([150, 170])
  
  # print("apex: ", apex_coor)
  
  if apex_coor and tight_turn == 0:
    # print("apex coor ", apex_coor[0][0][0])
    cv2.drawMarker(detected_both, (apex_coor[0][0][0], apex_coor[0][0][1]), (0,200,200), cv2.MARKER_CROSS, 10,1,1)
    cv2.putText(detected_both, "APEX", (apex_coor[0][0][0], apex_coor[0][0][1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,10), 2)
  if tight_turn == 1:
    print("Tight Turn Right")
    cv2.putText(detected_both, "Tight Turn Right", (virtual_apex_coor[0][0], virtual_apex_coor[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,10), 2)
    # cv2.arrowedLine(detected_both, )
  if tight_turn == -1:
    print("Tight Turn Left")
    cv2.putText(detected_both, "Tight Turn Left", (virtual_apex_coor[0][0], virtual_apex_coor[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,10), 2)
    # cv2.arrowedLine(detected_both, )

  return apex_coor, side

# def extreme_simple_apex(detected_both, yellow_cone_connect_sequence, red_cone_connect_sequence):
