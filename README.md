# Source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation"

This repository contains the source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation" published in the IEEE Robotics and Automation Letters ([DOI: 10.1109/LRA.2023.3333233](https://doi.org/10.1109/lra.2023.3333233)).

<p align="center">
  <img src="/res/img/teaser.gif" alt="Ground segmentation results"/>
</p>

## Dependencies

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

## Development Container

To be able to use Nvidia GPU graphics within Docker follow container toolkit installation steps <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-the-nvidia-container-toolkit>.

## Build

Install required dependencies

```bash
pip3 install -r requirements.txt
```

Build:

```bash
catkin build -DCMAKE_BUILD_TYPE=Release groundgrid
```

## Clean

```bash
catkin clean
```

## Launch

### Playback

```bash
source devel/setup.bash
roslaunch groundgrid KITTIPlayback.launch directory:=/path/to/the/SemanticKITTI dataset sequence:=0
```

The launch file opens a RViz window which displays the segmentation results:
<p align="center">
  <img src="/res/img/rviz.png" alt="SemanticKitti playback Rviz window"/>
</p>

## Ground Segmentation Evaluation

```bash
roslaunch groundgrid KITTIEvaluate.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
```

This launch file evaluates the ground segmentation performance of GroundGrid and displays the results every 500 processed clouds.
The final results are displayed upon receiving Ctrl+C in the terminal:

Received 4540 point clouds. KITTI sequence 00.

| Label                | Nonground % | Ground % | Nonground Total | Total       |
| -------------------- | ----------- | -------- | --------------- | ----------- |
| unlabeled            | 88.74%      | 11.26%   | 6,512,861       | 7,339,364   |
| outlier              | 42.51%      | 57.49%   | 121,616         | 286,056     |
| car                  | 94.42%      | 5.58%    | 43,078,880      | 45,622,648  |
| bicycle              | 89.85%      | 10.15%   | 200,919         | 223,610     |
| motorcycle           | 95.39%      | 4.61%    | 500,522         | 524,684     |
| truck                | 97.61%      | 2.39%    | 416,124         | 426,308     |
| other-vehicle        | 96.34%      | 3.66%    | 1,199,946       | 1,245,564   |
| person               | 95.95%      | 4.05%    | 68,227          | 71,104      |
| bicyclist            | 100.00%     | 0.00%    | 5               | 5           |
| motorcyclist         | 0.00%       | 100.00%  | 0               | 8           |
| road                 | 0.07%       | 99.93%   | 68,465          | 95,649,669  |
| parking              | 0.45%       | 99.55%   | 37,828          | 8,450,594   |
| sidewalk             | 0.91%       | 99.09%   | 716,154         | 78,601,664  |
| other-ground         | 6.43%       | 93.57%   | 192             | 2,985       |
| building             | 97.33%      | 2.67%    | 117,586,234     | 120,810,401 |
| fence                | 88.91%      | 11.09%   | 15,821,127      | 17,793,867  |
| other-structure      | 89.92%      | 10.08%   | 713,791         | 793,778     |
| lane-marking         | 0.16%       | 99.84%   | 171             | 109,456     |
| vegetation           | 93.43%      | 6.57%    | 121,595,505     | 130,139,604 |
| trunk                | 97.88%      | 2.12%    | 4,495,649       | 4,592,878   |
| terrain              | 6.68%       | 93.32%   | 1,939,107       | 29,038,187  |
| pole                 | 98.14%      | 1.86%    | 1,819,715       | 1,854,290   |
| traffic-sign         | 99.87%      | 0.13%    | 248,254         | 248,565     |
| other-object         | 89.59%      | 10.41%   | 4,577,201       | 5,109,077   |
| moving-car           | 96.48%      | 3.52%    | 245,225         | 254,183     |
| moving-bicyclist     | 98.23%      | 1.77%    | 155,263         | 158,054     |
| moving-person        | 96.51%      | 3.49%    | 85,024          | 88,099      |
| moving-motorcyclist  | 94.68%      | 5.32%    | 2,011           | 2,124       |
| moving-bus           | 98.26%      | 1.74%    | 902             | 918         |
| moving-other-vehicle | 94.24%      | 5.76%    | 34,090          | 36,175      |
| Precision            | 96.05%      |          | 209,090,638     | 8,607,231   |
| Recall               | 98.70%      |          | 209,090,638     | 2,761,917   |
| F1                   | 97.35%      |          | 8,607,231       | 2,761,917   |
| Accuracy             | 97.24%      |          | 400,339,747     | 411,708,895 |
| IoUg                 | 94.84%      |          |                 |             |
