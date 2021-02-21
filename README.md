# AI for Cone Detection by Autonomous Car
#### by Ho Kai Siu

## Background
This project is part of the research on Autonomous Guiding System for Formula Student Autonomous (the FSA), which is a driverless racing competition for university student. The other four parts of the research are: the ROS simulation, Real-time monitoring system, PCBs designs and STM32 control and the AI for Map Generation, which are the honours projects of other four students. It is hoped our team could combine these five projects into a big system for the autonomous racing car.
+
![Autonomous Guiding System for FSA](/Pic/BG.png)

## Inspiration
The research is inspired by Mr Chan Tai Wing’s project in 2017, which shows the potential of AI guiding system on car racing. The “instant cone pair” detection algorithm was adopted, which would be discussed in the next page. Furthermore, our team had discussed with the PolyU racing team before to address their needs on an autonomous racing car. It is wished that this project could provide the framework of the autonomous racing car to the Racing team for participating in the Formula SAE in China.

## "Instant Cone Pair" Detection
#### by Mr Chan
Concerning the “instant cone pair” detection algorithm, it only focus on the first cone pair in front of the car. The detection would be operated pair by pair, which means that, the car only detect for the next cone pair after passing through the detected pair. The midpoint of the cone pair is calculated and determined for the car to pass through. This approach would give some problems to the system. For example, bottlenecking the car travel velocity since the car drive through the midpoint of the cone pair, which don’t give the shortest travel time. Moreover, the system is unable to handle a 90-degree corner as stated in Chan’s project.
+
![90-degree Turning Corner [1]](/Pic/90corner.png)

## Car Racing
### *In car racing, travel time is the first and foremost goal for participants to win the game*

## To Reduce Travel Time
* In straight Path
  * Power of Engine (**Horsepower**)
* At Corner
  * Cutting Corner (**Racing Line**)
+
![Racing Line Example [2]](/Pic/RacingLine.png)

## Project Objectives
To design an artificial intelligence (AI) guiding system for autonomous racing car
* Detect the incoming pattern of the traffic cones
* Determine the apex of the corner
* Give the corresponding velocity and steering angle
+
![Camera View of the Racing Car](/Pic/cameraview.png)

## System Design
![System Flow](/Pic/systemdesign.png)
+
First, a stream of video frames are inputted into the system. Two HSV color masking would be operated to retrieve only the yellow and red color component in the frame, which are the cones colors in the left and right borders. Then, the masked frames are passed to the YOLO object detector to detect the cones in the frames. The centre coordinates of the cones are calculated and provided for the cone-to-path connection session to form the borderlines. After that, the apex in the borderlines would be detected. Finally, speed and steering angle for the micro-controller to drive the car across to apex would be sent.

## Hardware
The Nivida Jetson TX2 and a RGB camera were utilised. A Depth Camera, like RealSense D435i was used instead of RGB camera to give additional depth data of cone for more accurate cone-to-path connection

## Implementation
### HSV Color Masking
##### *with OpenCV*
Two ranges of HSV values were set for retrieving all the yellow and red color components in the frame. 
(*To further reduce the chance of false-class detection*)
![HSV trackbar](/Pic/hsvtrackbar.png)
*Trackbars for setting Lower (L) and Upper (U) H, S, V values*

### YOLO Object Detection
##### *YOLOv4*
![MC COCO Object Detection Comparison [3]](/Pic/yolov4.png)

It was trained with a custom training dataset (300 RGB images of cones with different scales). 
* 2 classes (*Yellow Cone, Red Cone*)
* 416x416 network size
* 64 branch size
* 32 subdivisions
* 6000 max_branch
* 3200, 3600 steps
* (2 classes + 5) x 3 = 21 filters before each YOLO layer

![Loss](/Pic/loss.jpeg)

Since two masked frames are passed to the YOLO, two arrays of centres coordinates (*Yellow Cone, Red Cone*) are calculated and returned for the next procedure.

![Detected Cone’s boundary box with centre points marker](/Pic/detectedcenter.png)

### Cone-to-Path (Cone Connection) Algorithm
##### *Transform unrelated coordinates of cone centers into related borders*
In the view of the car. The first pair of cones (**root cones**) possess the largest boundary box area and y-coordinate. The system filtered the detected cones by an area threshold of 600 and confident level of 0.6. The remaining cone centres were then connected in descending order of their y-coordinates.

Yet, the image would appear to converge at a vanishing point. Overlapping of cones near the point would occur, which degrade the cone connection. Therefore, this project would focus on the **first six cones only**.

![Connections of the First Six Cone](/Pic/connect6.png)

### Apex Detection
1. Calculate the slopes of the connected paths
  - $\frac{ya-yb}{xa-xb}$

