import cv2
import numpy as np 
import math
import darknet as dn
import pyrealsense2 as rs

def get_real_world_coordinates(depth_frame, x, y):
    # Intrinsics & Extrinsics
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        #color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        #depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
        #depth_image = np.asanyarray(depth_frame.get_data())
        #depth = depth_image[y][x]
    depth = depth_frame.get_distance(x,y)
    coordinates = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth)
    # print(f"The real world coordinates of the cone is {coordinates}")
    distance = math.sqrt(((coordinates[0])**2) + ((coordinates[1])**2) + ((coordinates[2])**2))
    # print("Distance from camera to pixel:", distance)
    # print(f"coordinates are {coordinates[0]} ,{coordinates[1]}, {coordinates[2]}")
    # print("Z-depth from camera surface to pixel surface:", depth)
    return coordinates

# def cone_localization(detections, depth_frame, color_image, colors):
#     # Convert images to numpy arrays
#     depth_image = np.asanyarray(depth_frame.get_data())
#     for label, confidence, bbox in detections:
#         xmin, ymin, xmax, ymax = dn.bbox2points_(bbox)
#         min = [10000,0,0]
#         #x = math.floor(x)
#         #y = math.floor(y)
#         filtered_depth_img = cv2.GaussianBlur(depth_image, (3,3), 2, None, 2, cv2.BORDER_REPLICATE)
#         depth_bbox = filtered_depth_img[ymin:ymax,xmin:xmax]
#         print(depth_bbox)
#         arr = np.array(depth_bbox)
#         depth_median = np.median(arr)

#         for y_bbox in range(len(depth_bbox)):
#             for x_bbox in range(len(depth_bbox[0])):
#                 abs_diff = abs(depth_median - depth_bbox[y_bbox][x_bbox])
#                 if abs_diff < min[0]:
#                     min = [abs_diff, y_bbox, x_bbox]

#         # print (f"x coordinate is {xmin} + {min[2]}")
#         # print (f"y coordinate is {ymin} + {min[1]}")
#         if xmin <0 or ymin<0:
#             xmin = 0
#             ymin = 0
#         coordinates = get_real_world_coordinates(depth_frame, xmin + min[2], ymin + min[1]) 
#         '''
#         print(f"For dected {label} with confidence {confidence}, Depth at x={xmin + min[2]}, y={ymin + min[1]} is {depth_image[ymin + min[1]][xmin + min[2]]}")
#         print("The real world coordinates of the detected cone is :")
#         print(coordinates)
#         '''
#         #cv2.putText(img,f'{depthimg[y][x]/10}cm', (x+round(w/2),y+round(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[label], 1)
#         #cv2.putText(color_image,f'{coordinates[0]}', (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[label], 1)
#         #cv2.putText(color_image,f'{coordinates[2]}', (x,y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[label], 1)
#         '''
#         cv2.putText(color_image,f'{coordinates[0]}', (xmin + min[2],ymin + min[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[label], 1)
#         cv2.putText(color_image,f'{coordinates[2]}', (xmin + min[2],ymin + min[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[label], 1)
#         print(min)
#         print(f"The distance of the cone is {coordinates[2]*100}cm")
#         cv2.circle(color_image, (xmin + min[2],ymin + min[1]), radius=2, color=colors[label], thickness=-1)
#         '''
#         #cv2.imshow("color_image", color_image)
#     return color_image

# def cone_localization_(detection, depth_frame):
#     # Convert images to numpy arrays
#     depth_image = np.asanyarray(depth_frame.get_data())
#     for label, confidence, bbox in detections:
#         xmin, ymin, xmax, ymax = dn.bbox2points_(bbox)
#         min = [10000,0,0]
#         #x = math.floor(x)
#         #y = math.floor(y)
#         filtered_depth_img = cv2.GaussianBlur(depth_image, (3,3), 2, None, 2, cv2.BORDER_REPLICATE)
#         depth_bbox = filtered_depth_img[ymin:ymax,xmin:xmax]
#         # print(depth_bbox)
#         arr = np.array(depth_bbox)
#         depth_median = np.median(arr)

#         for y_bbox in range(len(depth_bbox)):
#             for x_bbox in range(len(depth_bbox[0])):
#                 abs_diff = abs(depth_median - depth_bbox[y_bbox][x_bbox])
#                 if abs_diff < min[0]:
#                     min = [abs_diff, y_bbox, x_bbox]

#         if xmin <0 or ymin<0:
#             xmin = 0
#             ymin = 0
#         coordinates = get_real_world_coordinates(depth_frame, xmin + min[2], ymin + min[1]) 
#     return coordinates