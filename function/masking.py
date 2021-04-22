import cv2
import numpy as np 
import os

def create_mask(hsvframe, low_hsv, high_hsv):
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.inRange(hsvframe, low_hsv, high_hsv)
    mask - cv2.erode(mask, kernel)
    
    return mask

def nothing(x):
    pass

cv2.namedWindow("Yellow Cone")
cv2.createTrackbar("L-H", "Yellow Cone", 11, 180, nothing)
cv2.createTrackbar("L-S", "Yellow Cone", 197, 255, nothing)
cv2.createTrackbar("L-V", "Yellow Cone", 0, 255, nothing)
cv2.createTrackbar("U-H", "Yellow Cone", 37, 180, nothing)
cv2.createTrackbar("U-S", "Yellow Cone", 255, 255, nothing)
cv2.createTrackbar("U-V", "Yellow Cone", 255, 255, nothing)

cv2.namedWindow("Red Cone")
cv2.createTrackbar("L-H", "Red Cone", 0, 180, nothing)
cv2.createTrackbar("L-S", "Red Cone", 197, 255, nothing)
cv2.createTrackbar("L-V", "Red Cone", 0, 255, nothing)
cv2.createTrackbar("U-H", "Red Cone", 180, 180, nothing)
cv2.createTrackbar("U-S", "Red Cone", 255, 255, nothing)
cv2.createTrackbar("U-V", "Red Cone", 255, 255, nothing)

image = cv2.imread("/Volumes/HenrySSD/Polyu_EIE/FYP/Screen Cap/imwrite/gazebo_demo/start_point/original.jpg")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

while True:
    y_l_h = cv2.getTrackbarPos("L-H", "Yellow Cone")
    y_l_s = cv2.getTrackbarPos("L-S", "Yellow Cone")
    y_l_v = cv2.getTrackbarPos("L-V", "Yellow Cone")
    y_u_h = cv2.getTrackbarPos("U-H", "Yellow Cone")
    y_u_s = cv2.getTrackbarPos("U-S", "Yellow Cone")
    y_u_v = cv2.getTrackbarPos("U-V", "Yellow Cone")

    r_l_h = cv2.getTrackbarPos("L-H", "Red Cone")
    r_l_s = cv2.getTrackbarPos("L-S", "Red Cone")
    r_l_v = cv2.getTrackbarPos("L-V", "Red Cone")
    r_u_h = cv2.getTrackbarPos("U-H", "Red Cone")
    r_u_s = cv2.getTrackbarPos("U-S", "Red Cone")
    r_u_v = cv2.getTrackbarPos("U-V", "Red Cone")

    #yellow hsv range
    low_yellow = np.array([y_l_h, y_l_s, y_l_v])
    high_yellow = np.array([y_u_h, y_u_s, y_u_v])

    #red hsv range
    low_red = np.array([r_l_h, r_l_s, r_l_v])
    high_red = np.array([r_u_h, r_u_s, r_u_v])

    #create masks of red and yellow cones seperately
    mask_yellow = create_mask(hsv, low_yellow, high_yellow)
    mask_red = create_mask(hsv, low_red, high_red)
    mask_red = mask_red - mask_yellow

    masked_yellow = cv2.bitwise_and(image,image, mask=mask_yellow)
    masked_red = cv2.bitwise_and(image,image, mask=mask_red)

    #capture results
    time_for_capture = 0
    k = cv2.waitKey(33)
    if k == ord('a'): #press a to cap
        time_for_capture = 1
        if time_for_capture == 1:
            time_for_capture = 0
            path_dir = "/Volumes/HenrySSD/Polyu_EIE/FYP/Final/Pic/hsv/"
            cv2.imwrite(os.path.join(path_dir, "yellow_mask.jpg"), mask_yellow)
            cv2.imwrite(os.path.join(path_dir, "red_mask.jpg"), mask_red)
            cv2.imwrite(os.path.join(path_dir, "masked_yellow.jpg"), masked_yellow)
            cv2.imwrite(os.path.join(path_dir, "masked_red.jpg"), masked_red)
    
    cv2.imshow("masked_yellow", masked_yellow)
    cv2.imshow("masked_red", masked_red)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break