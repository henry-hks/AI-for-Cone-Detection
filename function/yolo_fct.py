import cv2
import numpy as np 
import system_filter as sf 
import sys
sys.path.append("/home/fyp/darknet")
import darknet as dn 
import darknet_images as dni

# def get_detection(image, mask, net, class_colors, name):
#   name = [name]
#   image_for = image
#   new_image = cv2.bitwise_and(image_for,image_for, mask=mask)
#   image_for, detections = dni.image_detection_(new_image, net, name, class_colors, 0.5)
#   return detections

#low pass filter
T = 5.0       #Sample period
fs = 30.0     #Sample Rate (Hz)
cutoff = 2    #Cutoff frequency of the filter (Hz)
nyq = 0.5*fs  #Nyquist rate
order = 2     #order of LPF
n = int(T*fs) #Total number of samples

def cone_detect(detected_both, image, mask, net, class_colors, name):
    name = [name]
    image_for = image
    new_image = cv2.bitwise_and(image_for,image_for, mask=mask)
    image_for, detections = dni.image_detection_(new_image, net, name, class_colors, 0.5)

    #low pass filtering
    detections_np = np.array(detections)
    LPF_detections = sf.butter_lowpass_filter(detections_np, cutoff, fs, order, nyq)
    print(" LPF_detections: ", LPF_detections)

    detected = image_for
    yolo_cones_center = []
    for label, confidence, bbox in detections:
      left, top, right, bottom = dn.bbox2points_(bbox)
      area = (right-left)*(bottom-top)
      if area > 600:# and confidence > 0.6:
        cv2.rectangle(detected, (left, top), (right, bottom), (255,0,100), 1)
        cv2.putText(detected, "{} [{:.2f}]".format(label, float(confidence)),
                    (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255,0,100), 2)
        #draw on the resized original image
        cv2.rectangle(detected_both, (left, top), (right, bottom), (255,0,100), 1)
        cv2.putText(detected_both, "{} [{:.2f}]".format(label, float(confidence)),
                    (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255,0,100), 2)
        
        #get the centers of the bbox
        centerx = int((left + right)/2)
        centery = int((top + bottom)/2)
        yolo_cones_center.append([centerx,centery])
        cv2.drawMarker(detected, (centerx, centery), class_colors, cv2.MARKER_CROSS, 10,1,1 )
        
        #draw on the resized original image
        cv2.drawMarker(detected_both, (centerx, centery), class_colors, cv2.MARKER_CROSS, 10,1,1 )
    
    return detected_both, detected, yolo_cones_center, detections

def cone_connect(detected, detected_both, yolo_cones_center):
    yolo_cones_center = sorted(yolo_cones_center, key=lambda k:[k[1], k[0]], reverse=True)
    yolo_cones_center = yolo_cones_center[:3]
    yolo_slopes = []
    for i in range(len(yolo_cones_center)):
      if i != len(yolo_cones_center)-1:
        if yolo_cones_center[i][1] > yolo_cones_center[i+1][1]:
          
          cv2.line(detected, (yolo_cones_center[i][0],yolo_cones_center[i][1]) , (yolo_cones_center[i+1][0], yolo_cones_center[i+1][1]), (255,0,0), 1)

          #draw on the resized original image
          cv2.line(detected_both, (yolo_cones_center[i][0],yolo_cones_center[i][1]) , (yolo_cones_center[i+1][0], yolo_cones_center[i+1][1]), (255,0,0), 1)

          #get the slopes between yellow cones
          xdiff = (yolo_cones_center[i][0]) - (yolo_cones_center[i+1][0])
          ydiff = (yolo_cones_center[i][1]) - (yolo_cones_center[i+1][1])
          if xdiff != 0:
            sl = round(np.true_divide(ydiff, xdiff, out=None), 5)
          else:
            sl = 999
          yolo_slopes.append(sl)
    
    return detected_both, detected, yolo_slopes