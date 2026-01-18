# Copyright 2025 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin
# CC BY-NC-SA 4.0
import os

import numpy as np
import math
import tf_transformations
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

	# map odom transform (identity transform)
	map_odom_tf = launch_ros.actions.Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='map_odom_tf_pub',
		arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

	rviz_config_dir = PathJoinSubstitution([FindPackageShare('groundgrid'), 'param/groundgrid.rviz'])
	rviz2 = launch_ros.actions.Node(
		package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
	)

	launch_dir = PathJoinSubstitution([FindPackageShare('groundgrid'), 'launch'])

	launch_description = launch.LaunchDescription([
	    IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'GroundGrid.launch.py']),
        ),
	    map_odom_tf,
	    rviz2
	])

	return launch_description

if __name__ == '__main__':
	generate_launch_description()
