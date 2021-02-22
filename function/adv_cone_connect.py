import cv2
import numpy as np 
import fuzzy
import coordinates
import math
import sys
sys.path.append("/home/fyp/darknet")
import darknet as dn 
import darknet_images as dni 


def getDepth_dcm(detections, depth_frame, depthimg, img, colors):
    center_with_depth = []
    for label, confidence, bbox in detections:
        x, y, w, h = bbox

        print("bbox: ", bbox)
        x = int(round(x))
        y = math.floor(y)
        w = int(round(w))
        h = int(round(h))
        # print("For dected {label} with confidence {confidence} , Depth at x={x} , y={y} is {depthimg[y][x]}".format(label=label, confidence=float(confidence),x=x, y=y, depthimg=depthimg))
        print("For detected ", label, "with confidence ", confidence, ", Depth at x= ", x , ", y= is", depthimg[int(y)][int(x)])
        # print("depth image of the boundary box is :")
        depth_bbox = depthimg[int(y-round(h/2)):int(y+round(h/2)),int(x-round(w/2)):int(x+round(w/2))]
        # print(depth_bbox)
        arr = np.array(depth_bbox)
        depth_median = np.median(arr)
        dm_d10 = np.round(depth_median/10,2)
        
        #cv2.putText(img,f'{depthimg[y][x]/10}cm', (x+round(w/2),y+round(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[label], 1)
        cv2.putText(img,"{} cm".format(dm_d10), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,200,200), 1)
        print("The distance of the cone is ", dm_d10, "cm")
        # cv2.circle(img, (x,y), radius=1, color=(255,0,100), thickness=-1)
        cv2.drawMarker(img, (x, y), (255,200,200), cv2.MARKER_CROSS, 10,1,1 )

        # #cone localize
        xmin, ymin, xmax, ymax = dn.bbox2points_(bbox)
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
        # coor = coordinates.get_real_world_coordinates(depth_frame, x, y) 
        center_with_depth.append([[x,y], dm_d10, coor])

    center_with_depth = sorted(center_with_depth, key=lambda k:[k[1], k[0][0]], reverse=False)
    print("center with depth", center_with_depth)
    return img, center_with_depth

def drawline_sequence(detected_both, cone_connect_sequence):
  cv2.line(detected_both, (cone_connect_sequence[0][0][0], cone_connect_sequence[0][0][1]) , (cone_connect_sequence[1][0][0], cone_connect_sequence[1][0][1]), (255,100,255), 2)
  cv2.line(detected_both, (cone_connect_sequence[1][0][0], cone_connect_sequence[1][0][1]) , (cone_connect_sequence[2][0][0], cone_connect_sequence[2][0][1]), (255,255,100), 2)

def cal_slopes(cone_connect_sequence):
  yolo_slopes = []
  for i in range(len(cone_connect_sequence)):
    if i != len(cone_connect_sequence)-1:
      #get the slopes between yellow cones
      xdiff = (cone_connect_sequence[i][0][0]) - (cone_connect_sequence[i+1][0][0])
      ydiff = (cone_connect_sequence[i][0][1]) - (cone_connect_sequence[i+1][0][1])
      if xdiff != 0:
        sl = round(np.true_divide(ydiff, xdiff, out=None), 5)
      else:
        sl = 999
      yolo_slopes.append(sl)
  return yolo_slopes

def cone_connect_with_depth(detected_both, center_with_depth, color_id): #color_id: 0 for yellow, 1 for red
  cone_connect_sequence = []
  direction = 0
  if len(center_with_depth) >= 3:
    depth_difference_1 = center_with_depth[1][1] - center_with_depth[0][1] 
    depth_difference_2 = center_with_depth[2][1] - center_with_depth[1][1]
    print("depth diferences: ", depth_difference_1, depth_difference_2)
    if depth_difference_1 > 0.01 and depth_difference_2 > 0.01: #if cones are separated obviously
      cone_connect_sequence=[center_with_depth[0], center_with_depth[1], center_with_depth[2]]
      
    elif depth_difference_1 < 0.01 and depth_difference_2 > 0.01: #first two cones are horizontally aligned
      if center_with_depth[0][0][0] <= center_with_depth[2][0][0] <= center_with_depth[1][0][0] or center_with_depth[0][0][0] >= center_with_depth[2][0][0] >= center_with_depth[1][0][0]: 
        #if the further cone is horizontally between two near aligned cone pair
        if color_id == 0: # connect for yellow, probably sharp turn left
          cone_connect_sequence = [center_with_depth[1], center_with_depth[2], center_with_depth[0]]
          
        elif color_id == 1: #connect for red, probably sharp turn right
          cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]
          
      elif center_with_depth[2][0][0] <= center_with_depth[1][0][0] <= center_with_depth[0][0][0]:
        #connect for both, probably 90 degree turn left
        cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]

      elif center_with_depth[2][0][0] >= center_with_depth[1][0][0] >= center_with_depth[0][0][0]:
        #connect for both, probably 90 degree turn left
        cone_connect_sequence = [center_with_depth[2], center_with_depth[1], center_with_depth[0]]
    
    elif depth_difference_2 < 0.01 and depth_difference_1 > 0.01: #last two cones are horizontally aligned
      if center_with_depth[2][0][0] <= center_with_depth[0][0][0] <= center_with_depth[1][0][0] or center_with_depth[2][0][0] >= center_with_depth[0][0][0] >= center_with_depth[1][0][0]: 
        #if the further cone is horizontally between two near aligned cone pair
        if color_id == 0: # connect for yellow, probably sharp turn left
          cone_connect_sequence = [center_with_depth[0], center_with_depth[2], center_with_depth[1]]
          
        elif color_id == 1: # connect for red, probably sharp turn left
          cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
          
      elif center_with_depth[2][0][0] <= center_with_depth[1][0][0] <= center_with_depth[0][0][0]:
        #connect for both, turn left
        cone_connect_sequence = [center_with_depth[0], center_with_depth[1], center_with_depth[2]]
        
      elif center_with_depth[2][0][0] >= center_with_depth[1][0][0] >= center_with_depth[0][0][0]:
        #connect for both, turn right
        cone_connect_sequence = [center_with_depth[2], center_with_depth[1], center_with_depth[0]]
    
    elif depth_difference_1 < 0.01 and depth_difference_2 < 0.01: #three cones are horizontally aligned
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
  
  yolo_slopes=[]
  if cone_connect_sequence:
    #draw lines
    drawline_sequence(detected_both, cone_connect_sequence)
    #calculate slopes
    yolo_slopes.append(cal_slopes(cone_connect_sequence))

  return detected_both, cone_connect_sequence, yolo_slopes

def direction_detect(yolo_slopes_yellow, yolo_slopes_red):
  direction = [0,0] # -2 = very left; -1 = left; 0 = straight; 1 = right; 2 = very right 

  #yellow
  #straight
  if yolo_slopes_yellow <= -0.2:
    direction[0]=0
  #left
  if -0.2 < yolo_slopes_yellow <= -0.01:
    direction[0]=-1
  #very left
  if -0.01 < yolo_slopes_yellow <=0:
    direction[0]=-2
  #very right
  if 0 < yolo_slopes_yellow <=0.01:
    direction[0]=2
  #right
  if 0.01 < yolo_slopes_yellow:
    direction[0]=1

  #red
  #right
  if yolo_slopes_yellow <= -0.01:
    direction[1]=1
  #very right
  if -0.01 < yolo_slopes_yellow <= 0:
    direction[0]=2
  #very left
  if 0 < yolo_slopes_yellow <= 0.01:
    direction[0]=-2
  #left
  if 0.01 < yolo_slopes_yellow <= 0.2:
    direction[0]=-1
  #straight
  if 0.2 < yolo_slopes_yellow:
    direction[0]=0

  return direction
    
def get_directions_array(yolo_slopes_yellow, yolo_slopes_red):
  directions = []
  if len(yolo_slopes_yellow) == 2 and len(yolo_slopes_red) == 2:
    for i in range(len(yolo_slopes_yellow)):
      direction = direction_detect(yolo_slopes_yellow[i], yolo_slopes_red[i])
      directions.append(direction)
  
  return directions

def apex_detect(detected_both, yellow_cone_connect_sequence, red_cone_connect_sequence, yolo_slopes_yellow, yolo_slopes_red, directions):
  apex = 0 # 0=no apex; 1=apex detected
  side = 0 # -1=left; 0=straight; 1=right
  apex_coor = []
  if len(directions) == 2:
    if -2 <= direction[0][0] <= -1 and -2 <= direction[0][1] <= -1 and -2 <= direction[1][0] <= -1 and -2 <= direction[1][1] <= -1:
      side = -1 #left
    if 1 <= direction[0][0] <= 2 and 1 <= direction[0][1] <= 2 and 1 <= direction[1][0] <= 2 and 1 <= direction[1][1] <= 2:
      side = 1 #right
  
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
  
  return apex_coor



