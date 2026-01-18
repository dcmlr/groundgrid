# Copyright 2025 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin
# CC BY-NC-SA 4.0
import os

import numpy as np
import math
import tf_transformations
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def setup(context):
	dataset_value = launch.substitutions.LaunchConfiguration('dataset_name').perform(context).lower()
	dataset_path = launch.substitutions.LaunchConfiguration('dataset_path').perform(context)
	poses_file = launch.substitutions.LaunchConfiguration('poses_file').perform(context)
	sequence_value = launch.substitutions.LaunchConfiguration('sequence').perform(context)
	sensor_value = launch.substitutions.LaunchConfiguration('sensor').perform(context)
	config_file = launch.substitutions.LaunchConfiguration('config_file').perform(context)

	if dataset_value == "kitti":
		# add leading zero
		if len(sequence_value) == 1:
			sequence_value = '0' + sequence_value

		if poses_file == '':
			poses_file = os.path.join(dataset_path, 'sequences', sequence_value, 'poses.txt')

		# fix for ros2 typecasting bug
		if sequence_value == '08' or sequence_value == '09':
			sequence_value = sequence_value[1:]
		
	if dataset_value == "kitti" or dataset_value == "kitti360":
		config_file = 'kitti.yaml'
	elif dataset_value == "mulran":
		config_file = 'mulran.yaml'
	elif dataset_value == "helipr":
		if sensor_value.lower() == "ouster":
			config_file = 'helipr_ouster.yaml'
		if sensor_value.lower() == "aeva":
			config_file = 'helipr_aeva.yaml'
		if sensor_value.lower() == "avia":
			config_file = 'helipr_avia.yaml'
		if sensor_value.lower() == "velodyne":
			config_file = 'helipr_velodyne.yaml'
	
	# GroundGrid parameters for the LiDAR
	# Select parameter file according to lidar sensor 
	config = os.path.join(
        get_package_share_directory('groundgrid'),
        'param',
        config_file
    )

	# check parameters
	if not os.path.isfile(config):
		print(f"ERROR: Could not find groundgrid config file: {config}, using defaults")
	if dataset_value != "live":
		if not os.path.isfile(poses_file):
			print(f"ERROR: Could not find poses file: {poses_file}")

	# Setup of the extrinsic sensor calibration transform
	if dataset_value == "helipr":
		if sensor_value.lower() == "ouster":
		    base_link_velodyne_tf = launch_ros.actions.Node(
		        package='tf2_ros',
		        executable='static_transform_publisher',
		        name='base_link_velodyne_tf_pub',
		        arguments=['0.0', '0.0', '2.0', '0.0', '0.0', '0.0', #TODO: this is a rough guess
		                   'base_link',
		                   'velodyne'])
		elif sensor_value.lower() == "aeva":
		    base_link_velodyne_tf = launch_ros.actions.Node(
		        package='tf2_ros',
		        executable='static_transform_publisher',
		        name='base_link_velodyne_tf_pub',
		        arguments=['0.0', '0.0', '2.0', '0.0', '-0.015595725680495854', '0.007271127000748835', # Aeva -> Ouster
		                   'base_link',
		                   'velodyne'])
		elif sensor_value.lower() == "avia":
		    base_link_velodyne_tf = launch_ros.actions.Node(
		        package='tf2_ros',
		        executable='static_transform_publisher',
		        name='base_link_velodyne_tf_pub',
		        arguments=['0.0', '0.0', '2.0', '0.0', '0.009958177597463428', '-0.017660507983823313', # Avia -> Ouster
		                   'base_link',
		                   'velodyne'])
		elif sensor_value.lower() == "velodyne":
		    base_link_velodyne_tf = launch_ros.actions.Node(
		        package='tf2_ros',
		        executable='static_transform_publisher',
		        name='base_link_velodyne_tf_pub',
		        arguments=['0.0', '0.0', '2.0', '0.0', '0.005542258713330074', '0.0056861114771429465', # Velodyne -> Ouster
		                   'base_link',
		                   'velodyne'])
	elif dataset_value == "kitti":
	    base_link_velodyne_tf = launch_ros.actions.Node(
	        package='tf2_ros',
	        executable='static_transform_publisher',
	        name='base_link_velodyne_tf_pub',
	        arguments=['0.0', '0.0', '1.733', '0.0', '0.0', '0.0', 
	                   'base_link',
	                   'velodyne'])
	elif dataset_value == "kitti360":
	    base_link_velodyne_tf = launch_ros.actions.Node(
	        package='tf2_ros',
	        executable='static_transform_publisher',
	        name='base_link_velodyne_tf_pub',
	        arguments=['0.771049336280387', '0.29854143649499193', '-0.8362802189143268', '0.005805702483432155', '-0.010400477715954315', '3.1385789123483367', 
	                   'base_link',
	                   'velodyne'])
	elif dataset_value == "mulran":
	    base_link_velodyne_tf = launch_ros.actions.Node(
	        package='tf2_ros',
	        executable='static_transform_publisher',
	        name='base_link_velodyne_tf_pub',
	        arguments=['0', '0.0', '0', '0', '-0.0222359877559', '0.000001745329', 
	                   'base_link',
	                   'velodyne'])
	else:
	    base_link_velodyne_tf = launch_ros.actions.Node(
	        package='tf2_ros',
	        executable='static_transform_publisher',
	        name='base_link_velodyne_tf_pub',
	        arguments=['0.0', '0.0', '2.0', '0.0', '0.0', '0.0', # default
	                   'base_link',
	                   'velodyne'])

	# For live mode only return GroundGrid node launch description, without transform publisher
	if dataset_value == "live":
		return [launch_ros.actions.Node(
	                                package='groundgrid',
									executable='groundgrid_node',
									name='groundgrid_node',
									output='screen',
									parameters=[
										{'groundgrid/dataset_name': launch.substitutions.LaunchConfiguration('dataset_name')},
										{'use_sim_time': launch.substitutions.LaunchConfiguration('simtime')},
										{'groundgrid/dataset_path': launch.substitutions.LaunchConfiguration('dataset_path')},
										{'groundgrid/sensor': launch.substitutions.LaunchConfiguration('sensor')},
										{'groundgrid/sequence': sequence_value},
										{'groundgrid/poses_file': poses_file},
										{'groundgrid/poses_delimiter': launch.substitutions.LaunchConfiguration('poses_file_delimiter')},
										{'groundgrid/frame_rate_cap': launch.substitutions.LaunchConfiguration('frame_rate_cap')},
										{'groundgrid/visualize': launch.substitutions.LaunchConfiguration('visualize')},
										{'groundgrid/evaluation': launch.substitutions.LaunchConfiguration('evaluation')},
										config
									],
									remappings=[
										('/pointcloud', launch.substitutions.LaunchConfiguration('pointcloud_topic')),
									('/groundgrid/odometry_in', launch.substitutions.LaunchConfiguration('odometry_topic')),
									]
		)]

	return [base_link_velodyne_tf,
		launch_ros.actions.Node(
                                package='groundgrid',
								executable='groundgrid_node',
								name='groundgrid_node',
								output='screen',
								parameters=[
									{'groundgrid/dataset_name': launch.substitutions.LaunchConfiguration('dataset_name')},
									{'use_sim_time': launch.substitutions.LaunchConfiguration('simtime')},
									{'groundgrid/dataset_path': launch.substitutions.LaunchConfiguration('dataset_path')},
									{'groundgrid/sensor': launch.substitutions.LaunchConfiguration('sensor')},
									{'groundgrid/sequence': sequence_value},
									{'groundgrid/poses_file': poses_file},
									{'groundgrid/poses_delimiter': launch.substitutions.LaunchConfiguration('poses_file_delimiter')},
									{'groundgrid/frame_rate_cap': launch.substitutions.LaunchConfiguration('frame_rate_cap')},
									{'groundgrid/visualize': launch.substitutions.LaunchConfiguration('visualize')},
									{'groundgrid/evaluation': launch.substitutions.LaunchConfiguration('evaluation')},
									config
								],
								remappings=[
									('/pointcloud', launch.substitutions.LaunchConfiguration('pointcloud_topic')),
									('/groundgrid/odometry_in', launch.substitutions.LaunchConfiguration('odometry_topic')),
								]
	)]


def generate_launch_description():
	# use sim time (for rosbag playbag)
	sim_time_arg = launch.actions.DeclareLaunchArgument(
		name='simtime',
		default_value='False',
		description='Whether to use simulated time or not'
	)

		# dataset name (kitti/helipr)
	dataset_name = launch.actions.DeclareLaunchArgument(
		name='dataset_name',
		default_value="kitti",
		description='Name of the dataset (KITTI, HeLIPR, ...). Use "live" for ros node mode'
	)

	# point cloud topic (only needed for ros node mode)
	point_cloud_topic = launch.actions.DeclareLaunchArgument(
		name="pointcloud_topic",
		default_value="/point_cloud",
		description="Point cloud topic to subscribe to"
	)

	# odometry topic (only needed for ros node mode)
	odometry_topic = launch.actions.DeclareLaunchArgument(
		name="odometry_topic",
		default_value="/groundgrid/odometry_in",
		description="Odometry topic to subscribe to"
	)

	# path to the dataset
	dataset_path = launch.actions.DeclareLaunchArgument(
		name="dataset_path",
		default_value='',
		description="Path to the dataset",
	)

	# dataset sequence (00, 01, etc for Kitti, bridge, town, roundabout for HeLiPR)
	sequence = launch.actions.DeclareLaunchArgument(
		name="sequence",
		default_value="00",
		description="Selected sequence of the dataset",
	)

	# dataset sensor (LiDAR type for HeLIPR: "Ouster", "Aeva", "Avia")
	sensor = launch.actions.DeclareLaunchArgument(
		name="sensor",
		default_value="velodyne",
		description="Selected sensor of the dataset",
	)

	# groundgrid config file
	config_file = launch.actions.DeclareLaunchArgument(
		name="config_file",
		default_value="kitti.yaml",
		description="Groundgrid sensor specific config file",
	)

	# odometry poses file: Txt file with poses in Kitti format
	odom_poses =launch.actions.DeclareLaunchArgument(
		name="poses_file",
		default_value='',
		description="Odometry poses file",
	)

	# delimiter for the pose files
	delim = launch.actions.DeclareLaunchArgument(
		name="poses_file_delimiter",
		default_value="' '",
		description="Odometry poses file delimiter",
	)

	# cap the processing frame rate to the specified frequency (0 = unlimited)
	frame_rate_cap = launch.actions.DeclareLaunchArgument(
		name="frame_rate_cap",
		default_value='0.0',
		description="Cap the processing frame rate to the specified frequency (0 = unlimited)",
	)

	# write the segmentation result into the intensity channel
	visualize = launch.actions.DeclareLaunchArgument(
		name="visualize",
		default_value='True',
		description="Write the segmentation result into the intensity channel, if False only non-ground points are being published",
	)

	# in evaluation mode groundgrid waits for the evaluation script with cloud publishing
	evaluation = launch.actions.DeclareLaunchArgument(
		name="evaluation",
		default_value='False',
		description="Enable evaluation mode",
	)

	opfunc = launch.actions.OpaqueFunction(function = setup)
	launch_description = launch.LaunchDescription([
		sim_time_arg,
		dataset_name,
		point_cloud_topic,
		odometry_topic,
		dataset_path,
		sequence,
		sensor,
		config_file,
		odom_poses,
		delim,
		frame_rate_cap,
		visualize,
		evaluation
	])

	launch_description.add_action(opfunc)
	return launch_description

if __name__ == '__main__':
	generate_launch_description()
