import cv2
import numpy as np 

def create_mask(hsvframe, low_hsv, high_hsv):
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.inRange(hsvframe, low_hsv, high_hsv)
    mask - cv2.erode(mask, kernel)
    
    return mask

def contour_center_detection(frame, mask, text):
    _, contours, _= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cones_center = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 160:
            if 12 >= len(approx) >= 3:
                cv2.drawContours(frame, [approx], 0, (255,0,0), 3) #(4,237,237)
                cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (4,237,237))
            if 20 >= len(approx) >= 13:
                cv2.drawContours(frame, [approx], 0, (255,200,0), 3) #(4,237,237)
                cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (100,237,237))

            #find center of yellow cone contours
            for c in [approx]:
              M = cv2.moments(c)
              if M["m00"] != 0:
                cx = int(M["m10"] / (M["m00"]))
                cy = int(M["m01"] / (M["m00"]))
                cones_center.append([cx,cy])
                cv2.drawMarker(frame, (cx,cy), (0,255,0), cv2.MARKER_CROSS, 5, 1, 1)
    
    return frame, cones_center

def connect_center_hsv(frame, cones_center):
    cones_center = sorted(cones_center, key=lambda k:[k[1], k[0]], reverse=True)
    cones_center = cones_center[:3]
    slopes = []
    for i in range(len(cones_center)):
      if i != len(cones_center)-1:
        if cones_center[i][1] > cones_center[i+1][1]:
          
          cv2.line(frame, (cones_center[i][0], cones_center[i][1]) , (cones_center[i+1][0], cones_center[i+1][1]), (255,0,0), 1)
        
          #get the slopes between cones
          xdiff = (cones_center[i][0]) - (cones_center[i+1][0])
          ydiff = (cones_center[i][1]) - (cones_center[i+1][1])
          if xdiff != 0:
            sl = round(np.true_divide(ydiff, xdiff, out=None), 5)
          else:
            sl = 999
          slopes.append(sl)

    return frame, slopes