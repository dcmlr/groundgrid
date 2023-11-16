# Source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation"
This repository contains the source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation" published in the IEEE Robotics and Automation Letters ([DOI: 10.1109/LRA.2023.3333233](https://doi.org/10.1109/lra.2023.3333233)).
<p align="center">
  <img src="/res/img/teaser.gif" alt="Ground segmentation results"/>
</p>

# Dependencies
- ROS Noetic Ninjemys
- catkin
- roscpp
- geometry_msgs
- sensor_msgs
- std_msgs
- message_generation
- message_runtime
- velodyne_pointcloud
- nodelet
- dynamic_reconfigure
- grid_map_core
- grid_map_ros
- grid_map_cv
- grid_map_loader
- grid_map_msgs
- grid_map_rviz_plugin
- grid_map_visualization
- cv_bridge
- pcl_ros

# Build
```
catkin build -DCMAKE_BUILD_TYPE=Release groundgrid
```

# Launch
## Playback
```
roslaunch groundgrid KITTIPlayback.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
```

The launch file opens a RViz window which displays the segmentation results:
<p align="center">
  <img src="/res/img/rviz.png" alt="SemanticKitti playback Rviz window"/>
</p>


## Ground Segmentation Evaluation
```
roslaunch groundgrid KITTIEvaluate.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
```

This launch file evaluates the ground segmentation performance of GroundGrid and displays the results every 500 processed clouds.
The final results are displayed upon receiving Ctrl+C in the terminal:
```
Stats
Received 4540 point clouds. KITTI sequence 00.
label			nonground %	ground %	nonground	total
unlabeled		88.74%		11.26%		6512861		7339364
outlier			42.51%		57.49%		121616		286056
car			94.42%		5.58%		43078880	45622648
bicycle			89.85%		10.15%		200919		223610
motorcycle		95.39%		4.61%		500522		524684
truck			97.61%		2.39%		416124		426308
other-vehicle		96.34%		3.66%		1199946		1245564
person			95.95%		4.05%		68227		71104
bicyclist		100.00%		0.00%		5		5
motorcyclist		0.00%		100.00%		0		8
road			0.07%		99.93%		68465		95649669
parking			0.45%		99.55%		37828		8450594
sidewalk		0.91%		99.09%		716154		78601664
other-ground		6.43%		93.57%		192		2985
building		97.33%		2.67%		117586234	120810401
fence			88.91%		11.09%		15821127	17793867
other-structure		89.92%		10.08%		713791		793778
lane-marking		0.16%		99.84%		171		109456
vegetation		93.43%		6.57%		121595505	130139604
trunk			97.88%		2.12%		4495649		4592878
terrain			6.68%		93.32%		1939107		29038187
pole			98.14%		1.86%		1819715		1854290
traffic-sign		99.87%		0.13%		248254		248565
other-object		89.59%		10.41%		4577201		5109077
moving-car		96.48%		3.52%		245225		254183
moving-bicyclist	98.23%		1.77%		155263		158054
moving-person		96.51%		3.49%		85024		88099
moving-motorcyclist	94.68%		5.32%		2011		2124
moving-bus		98.26%		1.74%		902		918
moving-other-vehicle	94.24%		5.76%		34090		36175
Precision		96.05%		209090638	8607231
Recall			98.70%		209090638	2761917
F1			97.35%		8607231		2761917
Accuracy		97.24%		400339747	411708895
IoUg			94.84%
```

