# Source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation"
This repository contains the ROS2 port of the source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation" published in the IEEE Robotics and Automation Letters ([DOI: 10.1109/LRA.2023.3333233](https://doi.org/10.1109/lra.2023.3333233)).
<p align="center">
  <img src="/res/img/teaser.gif" alt="Ground segmentation results"/>
</p>

# Dependencies
- ROS2 Jazzy Jalisco
- ament_cmake
- roscpp
- rclpy
- geometry_msgs
- sensor_msgs
- std_msgs
- velodyne_pointcloud
- nodelet
- dynamic_reconfigure
- grid_map_core
- grid_map_ros
- grid_map_cv
- grid_map_filters
- grid_map_loader
- grid_map_msgs
- grid_map_rviz_plugin
- grid_map_visualization
- cv_bridge
- pcl_ros
- tf_transformations

For printing the evaluation data: [PrettyTable](https://pypi.org/project/prettytable/). Ubuntu package: [python3-prettytable](https://answers.launchpad.net/ubuntu/jammy/+package/python3-prettytable)

# Installation
## Existing ROS2 Workspace
Clone this branch of the GroundGrid repository into your src folder. Then execute:
```
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build 
```
## Docker
Build with Docker:
```
docker build . -t groundgrid_docker_image
```

Run in Docker:
```
docker run -it --net=host --privileged --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --volume="${XAUTHORITY:-$HOME/.Xauthority}:/home/ubuntu/.Xauthority:rw" --volume="/Path/to/your/datasets:/var/datasets" --ipc=host --pid=host -e=DISPLAY -e=ROS_DOMAIN_ID groundgrid_docker_image:latest
```
Replace the volume "/path/to/your/datasets" with the correct directory to access datasets stored on the host machine.

# Dataset Preparations
## SemanticKITTI Dataset
Folder structure should follow the SemanticKITTI [website](https://semantic-kitti.org/dataset.html). The SemanticKITTI pose files are expected to be located in their respective sequence folders.

## HeLiPR Dataset
The "dataset_path" parameter should point to a directory containing the folders for the individual sequences:
```
HeLiPR/
├── Bridge01/
├── Roundabout03/
└── Town03/
```
The structure of the HeLiPR dataset sequences should follow the layout shown [here](https://sites.google.com/view/heliprdataset/system).

# Launch
## Playback of the KITTI Dataset
Launch playback of sequence 00 at 10 frames per second.
```
ros2 launch groundgrid KITTIPlayback.launch.py directory:=/path/to/the/SemanticKITTI/dataset sequence:=0 frame_rate_cap:=10.0
```

Set the parameter frame_rate_cap to 0.0 to process the scans as fast as possible.

The launch file opens an RViz window that displays the segmentation results:
<p align="center">
  <img src="/res/img/rviz.png" alt="SemanticKitti playback Rviz window"/>
</p>

## Playback of the HeLiPR Dataset
Launch playback of the Town01 sequence using the Livox Avia sensor at 10 frames per second. Other valid sensor keywords are "Aeva", "Ouster", and "Velodyne". They must match the folder names of the pointcloud data.
```
ros2 launch groundgrid HeLiPRPlayback.launch.py dataset_path:=/var/rosbags/HeLiPR/ sequence:="Town01" dataset_name:="helipr" sensor:="Avia" frame_rate_cap:=10.0  poses_file:=/var/rosbags/HeLiPR/Town01/LiDAR_GT/Avia_gt.txt
```

Set the parameter frame_rate_cap to 0.0 to process the scans as fast as possible.

## Run as ROS2-Node
Launch GroundGrid as a ROS2-Node, listening to the pointcloud topic "my_points" and the odometry topic "my_odom", using the Livox Avia parameters:
```
ros2 launch groundgrid Live.launch.py  dataset_name:="live" sensor:="Avia" pointcloud_topic:="/my_points" odometry_topic:="/my_odom"
```
The transform "velodyne"<->"base_link" must be available. The odometry is expected to be the pose of the robot's base_link in world coordinates.

## Ground Segmentation Evaluation
```
ros2 launch groundgrid KITTIEvaluate.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
```

This launch file evaluates the ground segmentation performance of GroundGrid. The average runtime of GroundGrid is also displayed in the console window. Note that only the runtime of the GroundGrid algorithm is measured, excluding the runtime of the evaluation script.
The final results are displayed upon receiving Ctrl+C in the terminal:
```
Keyboard interrupt received
Stats
Received 4541 point clouds.
+-------------------------------------------------------------------------+
|                       Ground Segmentation Results                       |
+----------------------+--------------+----------+------------+-----------+
| Label                | Non-Ground % | Ground % | Non-Ground |     Total |
+----------------------+--------------+----------+------------+-----------+
| unlabeled            |       88.63% |   11.37% |    6507747 |   7342481 |
| outlier              |       42.26% |   57.74% |     120875 |    286060 |
| car                  |       94.42% |    5.58% |   43080840 |  45626882 |
| bicycle              |       89.90% |   10.10% |     201027 |    223610 |
| motorcycle           |       95.27% |    4.73% |     499876 |    524684 |
| truck                |       97.66% |    2.34% |     416352 |    426308 |
| other-vehicle        |       96.21% |    3.79% |    1198359 |   1245564 |
| person               |       95.85% |    4.15% |      68156 |     71104 |
| bicyclist            |      100.00% |    0.00% |          5 |         5 |
| motorcyclist         |        0.00% |  100.00% |          0 |         8 |
| road                 |        0.07% |   99.93% |      67563 |  95683897 |
| parking              |        0.44% |   99.56% |      37478 |   8453862 |
| sidewalk             |        0.91% |   99.09% |     717062 |  78628024 |
| other-ground         |        6.60% |   93.40% |        197 |      2985 |
| building             |       97.33% |    2.67% |  117600840 | 120828669 |
| fence                |       88.91% |   11.09% |   15820943 |  17794237 |
| other-structure      |       89.91% |   10.09% |     714983 |    795249 |
| lane-marking         |        0.16% |   99.84% |        174 |    110509 |
| vegetation           |       93.44% |    6.56% |  121633668 | 130166727 |
| trunk                |       97.90% |    2.10% |    4497615 |   4594070 |
| terrain              |        6.65% |   93.35% |    1931381 |  29041151 |
| pole                 |       98.12% |    1.88% |    1819938 |   1854822 |
| traffic-sign         |       99.88% |    0.12% |     248357 |    248667 |
| other-object         |       89.56% |   10.44% |    4576816 |   5110304 |
| moving-car           |       96.59% |    3.41% |     245512 |    254183 |
| moving-bicyclist     |       98.27% |    1.73% |     155313 |    158054 |
| moving-person        |       96.41% |    3.59% |      84938 |     88099 |
| moving-motorcyclist  |       94.80% |    5.20% |       2097 |      2212 |
| moving-bus           |       98.26% |    1.74% |        902 |       918 |
| moving-other-vehicle |       98.91% |    1.09% |      35780 |     36175 |
+----------------------+--------------+----------+------------+-----------+
+--------------------------------------------+
|              Overall Results               |
+-----------+--------+-----------+-----------+
|    Metric | Result |   Correct | Incorrect |
+-----------+--------+-----------+-----------+
| Precision | 96.04% | 209166573 |   8615175 |
|    Recall | 98.70% | 209166573 |   2753855 |
|        F1 | 97.35% |           |           |
|  Accuracy | 97.24% | 400435222 |  11369030 |
|      IoUg | 94.84% | 209166573 |  11369030 |
+-----------+--------+-----------+-----------+
```

# Citation
```
@article{steinke2024groundgrid,
  author={Steinke, Nicolai and Goehring, Daniel and Rojas, Raúl},
  journal={IEEE Robotics and Automation Letters},
  title={GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation},
  year={2024},
  volume={9},
  number={1},
  pages={420-426},
  keywords={Sensors;Point cloud compression;Estimation;Laser radar;Image segmentation;Task analysis;Robot sensing systems;Range Sensing;Mapping;Field Robots},
  doi={10.1109/LRA.2023.3333233}}
```
