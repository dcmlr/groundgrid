#!/usr/bin/env python3
# Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
# and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or other materials provided
# with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
# endorse or promote products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, qos_profile_services_default
import csv
import time
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header, Empty
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2

import rospkg
import tf2_ros
import numpy as np
import os
#import ros2_numpy

import yaml
from prettytable import PrettyTable

############################# Code copied from ros2_numpy ##############################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jon Binney
# mappings between PointField types and numpy types

# prefix to the names of dummy fields we add to get byte alignment
# correct. this needs to not clash with any actual field names
DUMMY_FIELD_PREFIX = '__'

type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = int(np.prod(shape))
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(
                ('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_to_nptype[f.datatype].itemsize * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list

def get_xyz_points(cloud_array, remove_nans=True, dtype=float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
    a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & \
               np.isfinite(cloud_array['y']) & \
               np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (5,), dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']
    points[...,3] = cloud_array['intensity']
    points[...,4] = cloud_array['ring']

    return points

def pointcloud2_to_xyzir_array(cloud_msg, remove_nans=True):
    return get_xyz_points(
        pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)

def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the
    height is 1.

    The reason for using np.frombuffer rather than struct.unpack is
    speed... especially for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (
            fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

############################# End of code copied from ros2_numpy ##################################

class EvalNode(Node):
    def __init__(self):
        self.cloudCount = 0
        super().__init__('eval_groundpoint_classifier')
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        with open('src/groundgrid/cfg/semantic-kitti-all.yaml') as stream:
            try:
                self.CFG = yaml.safe_load(stream)
            except Exception as e:
                print(e)
                print("Error opening SemanticKITTI yaml configuration file.")
                self.CFG = []

        self.nonGroundPointLabelCount = dict()
        self.semanticCloudLabelCount = dict()
        self.truePositiveCloudLabelCount = dict()
        self.falsePositiveCloudLabelCount = dict()
        for label in self.CFG["labels"].values():
            self.nonGroundPointLabelCount[label] = 0
            self.semanticCloudLabelCount[label] = 0
            self.truePositiveCloudLabelCount[label] = 0
            self.falsePositiveCloudLabelCount[label] = 0

        self.truePositivOutlierCount = 0
        self.falsePositivOutlierCount = 0
        self.groundLabels = ["road", "sidewalk", "parking", "lane-marking"]
        self.additionalGroundLabels = ["other-ground", "terrain"]
        self.nonGroundLabels = ["bicycle", "moving-bicyclist", "motorcycle", "moving-motorcyclist", "person", "moving-person", "traffic-sign", "car", "moving-car",
                           "motorcyclist", "bicyclist", "truck", "moving-truck", "building", "fence", "trunk", "pole", "bus", "on-rails", "other-vehicle", "other-structure",
                           "other-object", "moving-on-rails", "moving-bus", "moving-other-vehicle"]
        self.publisher = self.create_publisher(Empty, '/groundgrid/next_cloud', qos_profile_services_default)

        self.subscription = self.create_subscription(PointCloud2,
                                 '/groundgrid/filtered_cloud',
                                 self.callback_predicted_cloud,
                                 qos_profile_services_default) # use reliable for offline processing, since best effort leads to data loss

        print('eval_groundpoint_classifier initialized')
        time.sleep(2)
        self.publisher.publish(Empty()) # let groundgrid know that we're ready

    def callback_predicted_cloud_old(self, cloud):
        for p in pointcloud2_to_array(cloud):
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            intensity = p[3]
            label = p[4]
            labelstring = self.CFG["labels"][label]

            if intensity == 99: # predicted obstacle
                self.nonGroundPointLabelCount[labelstring] += 1
            elif intensity == 49: # predicted ground
                if labelstring in self.groundLabels:
                    self.truePositiveCloudLabelCount[labelstring] += 1
                elif labelstring in self.additionalGroundLabels:
                    self.truePositiveCloudLabelCount[labelstring] += 1
                else:
                    self.falsePositiveCloudLabelCount[labelstring] += 1

            self.semanticCloudLabelCount[labelstring] += 1

        self.publisher.publish(Empty()) # let groundgrid know that we're ready

    def callback_predicted_cloud(self, cloud):
        self.cloudCount += 1

         # Read all points at once
        points = pointcloud2_to_xyzir_array(cloud)
    
        # Extract intensity and ring (label) columns
        intensities = points[:, 3]
        labels = points[:, 4]
    
        # Map numeric labels to label strings
        label_strings = np.vectorize(lambda x: self.CFG["labels"][x])(labels)
    
        # Vectorized counting for predicted obstacles (intensity == 99)
        obstacle_mask = intensities == 99
        unique_obstacle_labels, obstacle_counts = np.unique(label_strings[obstacle_mask], return_counts=True)
        for label, count in zip(unique_obstacle_labels, obstacle_counts):
            self.nonGroundPointLabelCount[label] += count
    
        # Vectorized counting for ground points
        ground_mask = intensities == 49
        ground_label_strings = label_strings[ground_mask]
    
        # True Positive and False Positive counting
        true_positive_mask = np.isin(ground_label_strings, self.groundLabels) | \
                             np.isin(ground_label_strings, self.additionalGroundLabels)
        false_positive_mask = ~true_positive_mask
    
        # Count true positives
        unique_true_positive_labels, true_positive_counts = np.unique(ground_label_strings[true_positive_mask], return_counts=True)
        for label, count in zip(unique_true_positive_labels, true_positive_counts):
            self.truePositiveCloudLabelCount[label] += count
    
        # Count false positives
        unique_false_positive_labels, false_positive_counts = np.unique(ground_label_strings[false_positive_mask], return_counts=True)
        for label, count in zip(unique_false_positive_labels, false_positive_counts):
            self.falsePositiveCloudLabelCount[label] += count
    
        # Semantic label counting
        unique_semantic_labels, semantic_counts = np.unique(label_strings, return_counts=True)
        for label, count in zip(unique_semantic_labels, semantic_counts):
            self.semanticCloudLabelCount[label] += count

        self.publisher.publish(Empty()) # let groundgrid know that we're ready
    
        return


    def print_statistics(self):
        print("Stats")
        print("Received " + str(self.cloudCount) + " point clouds.")
        table = PrettyTable()
        table.title = "Ground Segmentation Results"
        table.field_names = ["Label", "Non-Ground %", "Ground %", "Non-Ground", "Total"]
        for label in self.CFG["labels"].values():
            nonGroundPoints = self.nonGroundPointLabelCount[label]
            totalPoints = self.semanticCloudLabelCount[label]
            if totalPoints == 0:
                continue
            table.add_row([label, '{:2.2%}'.format(float(nonGroundPoints/totalPoints)), '{:2.2%}'.format(1.0-float(nonGroundPoints/totalPoints)), nonGroundPoints, totalPoints])

        truePgroundsum = 0
        truePgroundsumadd = 0
        trueNgroundNoVeg = 0
        falsePgroundObssumNoVeg = 0
        falseNgroundNoVeg = 0
        gtgroundsum = 0
        gtgroundsumadd = 0
        gtnonGroundSumNoVeg = 0
        for label in self.groundLabels:
            truePgroundsum += self.truePositiveCloudLabelCount[label]
            gtgroundsum += self.semanticCloudLabelCount[label]
            falseNgroundNoVeg += self.nonGroundPointLabelCount[label]

        truePgroundsumadd = truePgroundsum
        gtgroundsumadd = gtgroundsum
        for label in self.additionalGroundLabels:
            truePgroundsumadd += self.truePositiveCloudLabelCount[label]
            gtgroundsumadd += self.semanticCloudLabelCount[label]
            falseNgroundNoVeg += self.nonGroundPointLabelCount[label]

        for label in self.nonGroundLabels:
            falsePgroundObssumNoVeg += self.falsePositiveCloudLabelCount[label]
            gtnonGroundSumNoVeg += self.semanticCloudLabelCount[label]
            trueNgroundNoVeg += self.nonGroundPointLabelCount[label]


        # precision recall calculation (without vegetation)
        truePositiveGround = truePgroundsumadd
        trueNegativeGround = trueNgroundNoVeg
        falsePositiveGround = falsePgroundObssumNoVeg
        falseNegativeGround = falseNgroundNoVeg
        precision = truePositiveGround/(falsePositiveGround+truePositiveGround)
        recall = truePositiveGround/(falseNegativeGround+truePositiveGround)
        f1 = (2*truePositiveGround)/(2*truePositiveGround+falsePositiveGround+falseNegativeGround)
        accuracy = (truePositiveGround+trueNegativeGround)/(truePositiveGround+trueNegativeGround+falsePositiveGround+falseNegativeGround)

        table2 = PrettyTable()
        table2.title = "Overall Results"
        table2.align = 'r'
        table2.align['Metric'] = 'l'
        table2.field_names = ['Metric', 'Result', 'Correct', 'Incorrect']
        table2.add_row(['Precision', '{:2.2%}'.format(precision), truePositiveGround, falsePositiveGround])
        table2.add_row(['Recall', '{:2.2%}'.format(recall), truePositiveGround, falseNegativeGround])
        table2.add_row(['F1', '{:2.2%}'.format(f1), '', ''])
        table2.add_row(['Accuracy', '{:2.2%}'.format(accuracy), (truePositiveGround+trueNegativeGround), (falsePositiveGround+falseNegativeGround)])
        table2.add_row(['IoUg', '{:2.2%}'.format(truePositiveGround/(falsePositiveGround+gtgroundsumadd)), truePositiveGround, falseNegativeGround + falsePositiveGround])
        table.align = 'r'
        table.align['Label'] = 'l'
        print(table)
        print(table2)

if __name__ == '__main__':
    rclpy.init(args=None)
    eval_node = EvalNode()
    try:
        rclpy.spin(eval_node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    eval_node.print_statistics()
