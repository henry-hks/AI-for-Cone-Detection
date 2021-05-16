# AI for Cone Detection by Autonomous Car
### *by Ho Kai Siu, Henry*

## Background
<p align="justify">
This project is part of the research on Autonomous Guiding System for Formula Student Autonomous (the <b>FSA</b>), which is a driverless racing competition for university student. The research team consists of five students. The other four parts of the research are: <i>the ROS simulation, Real-time monitoring system, PCBs designs and STM32 control and the AI for Map Generation</i>, which are the honours projects of other four students. It is hoped our team could combine these five projects into a big system for the autonomous racing car.
</p>
<br>
<p align="center">
 <img src="/Pic/BG.png" width="500">
</p>

## Inspiration
<p align="justify">
The research is inspired by Mr Chan Tai Wing’s project [1] in 2017, which shows the potential of AI guiding system on car racing. The “instant cone pair” detection algorithm was adopted, which would be discussed in the next page. Furthermore, our team had discussed with the PolyU racing team before to address their needs on an autonomous racing car. It is wished that this project could provide the framework of the autonomous racing car to the Racing team for participating in the Formula SAE in China.
</p>

## "Instant Cone Pair" Detection (Old)
#### *by Mr Chan*
<p align="justify">
 <ul>
  <li>Focus on the first cone pair in front of the car</li>
  <li>The detection would be operated pair by pair</li>
  <li>The midpoint of the cone pair is calculated and determined for the car to pass through</li>
  <li><b>Limitations</b></li>
  <ul>
   <li>Bottlenecking the car travel velocity</li>
       Since the car drive through the midpoint of the cone pair, which don’t give the shortest travel time.
   <li>Inability of 90-degree corner determination and execution</li>
  </ul
</p>
<br>
<p align="center">
 <img src="/Pic/90corner.png" width="400">
 <br>
 <i>90-degree Turning Corner [1]</i>
</p>

## Car Racing
<p align="center">
 <b>In car racing, travel time is the first and foremost goal for participants to win the game</b>
 <br>
 Need <b>NEW</b> cone detection algorithm (This Project)
</p>

## To Reduce Travel Time
* In straight Path
  * Power of Engine (**Horsepower**)

* At Corner
  * Cutting Corner (**Racing Line**)
  <p align="center">
   <img src="/Pic/RacingLine.png" width="500">
   <br>
   <i>Racing Line Example [2]</i>
  </p>

## Project Objectives
To design an artificial intelligence (AI) guiding system for autonomous racing car
* Detect the incoming pattern of the traffic cones
* Determine the apex of the corner
* Give the corresponding velocity and steering angle

  <p align="center">
   <img src="/Pic/cameraview.png" width="400">
  </p>

<p align="center">
 <p align="center">
 <i>***<br/>
  In this project, it is assumed that the left boundary of the track is aligned with yellow cones, and the right border is formed by red cones.<br/>
  ***</i>
 </p>
</p>

## System Design

<p align="center">
 <img src="/Pic/systemdesign.png" width="700">
</p>

<p align="justify">
 <ol>
  <li>First, a stream of video frames are inputted into the system. Two HSV colour masking would be operated to retrieve only the yellow and red colour component in the frame, which are the cones colours in the left and right borders. </li>
  <li>Then, the masked frames are passed to the YOLO object detector to detect the cones in the frames. The centre coordinates of the cones are calculated.</li>
  <li>After that, the Real-world coordinates of the cones are obtained in the Real-world Coordinates Retrieval Stage.</li>
  <li>Next, the cone’s real-world coordinates are provided for the cone-to-path connection session to form the borderlines.</li>
  <li>Afterwards, the apex in the borderlines would be detected.</li>
  <li>Finally, speed and steering angle for the micro-controller to drive the car across to apex would be sent.</li>
 </ol>
</p>

## Hardware
<p align="justify">
The <b>Nivida Jetson TX2</b> and a <b>Depth Camera</b> were utilized. The RealSense D435i was used instead of RGB camera to give additional depth data of cone for more accurate cone-to-path connection.
</p>

## Implementation
### HSV Color Masking
##### *with OpenCV*
<p align="justify">
 Two ranges of HSV values were set for retrieving all the yellow and red color components in the frame. 
 <br>
 (<i>To further reduce the chance of false-class detection</i>)
</p>


<p align="center">
 <img src="/Pic/hsvtrackbar.png" width="600">

 <i>Trackbars for setting Lower (L) and Upper (U) H, S, V values</i>
</p>

### YOLO Object Detection
##### *YOLOv4*
<p align="center">
 <img src="/Pic/yolov4.png" width="400">
 <br>
 <i>MC COCO Object Detection Comparison [3]</i>
</p>


It was trained with a custom training dataset (300 RGB images of cones with different scales). 
* 2 classes (*Yellow Cone, Red Cone*)
* 416x416 network size
* 64 branch size
* 32 subdivisions
* 6000 max_branch
* 3200, 3600 steps
* (2 classes + 5) x 3 = 21 filters before each YOLO layer

<p align="center">
 <img src="/Pic/loss.jpeg" width="500">
 <br>
 <i>Loss</i>
</p>

<p align="justify">
Since two masked frames are passed to the YOLO, two arrays of centres coordinates (*Yellow Cone, Red Cone*) are calculated and returned for the next procedure.
</p>

<p align="center">
 <img src="/Pic/detectedcenter.png" width="100">
 <br>
 <i>Detected Cone’s boundary box with centre points marker</i>
</p>

### Cone-to-Path (Cone Connection) Algorithm
##### *Transform unrelated coordinates of cone centers into related borders*
<p align="justify">
 In the view of the car. The first pair of cones (<i>root cones</i>) possess the largest boundary box area and y-coordinate. The system filtered the detected cones by an area threshold of 600 and confident level of 0.6. The remaining cone centres were then connected in descending order of their y-coordinates.
<br>
Yet, the image would appear to converge at a vanishing point. Overlapping of cones near the point would occur, which degrade the cone connection. Therefore, this project would focus on the <b>first six cones only</b>.
</p>

<p align="center">
 <img src="/Pic/connect6.png" width="500">
 <br>
 <i>Connections of the First Six Cone</i>
</p>

### Apex Detection
<p align="justify">
 <ol>
  <li>Calculate the slopes of the connected paths</li>
   <p align="center">
    <img src="/Pic/slopef.png" width="300">
    <br>
    <i>Slope formula</i>
   </p>
  <li>Determine the direction of the path</li>
   <ul>
    <li>Route Type</li>
    <ul>
     <li>Straight Route</li>
     <li>Right-Turn Corner</li>
     <li>Left-Turn Corner</li>
    </ul>
    <li>Based on the sign combination of the slopes of two borders</li>
    <p align="center">
     <img src="/Pic/slope_route.png" width="400">
     <br>
     <i>Relationship between the borders' sign combination and the route type</i>
    </p>
   </ul>
  <li>Calculate the slopes' change</li>
  <li>Determine the apex</li>
 </ol>
</p>
