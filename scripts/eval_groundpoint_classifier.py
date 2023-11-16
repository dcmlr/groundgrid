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
import rospy
import csv
import time
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2, PointField

import rospkg
import tf2_ros
import tf
import numpy as np
import os

import yaml

from groundgrid.srv import NextCloud

def init():
    global tfBuffer, CFG, nonGroundPointLabelCount, semanticCloudLabelCount, truePositiveCloudLabelCount, falsePositiveCloudLabelCount, cloudCount, groundLabels, nonGroundLabels
    global additionalGroundLabels, truePositivOutlierCount, falsePositivOutlierCount, nextCloud
    cloudCount = 0
    rospy.init_node('eval_groundpoint_classifier')
    rospy.loginfo('eval_groundpoint_classifier initialized')
    rate = rospy.Rate(1) # hz
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        CFG = yaml.safe_load(open(rospkg.RosPack().get_path('groundgrid') + '/cfg/semantic-kitti-all.yaml', 'r'))
    except Exception as e:
        print(e)
        print("Error opening SemanticKITTI yaml configuration file.")
        CFG = []

    nonGroundPointLabelCount = dict()
    semanticCloudLabelCount = dict()
    truePositiveCloudLabelCount = dict()
    falsePositiveCloudLabelCount = dict()
    for label in CFG["labels"].values():
        nonGroundPointLabelCount[label] = 0
        semanticCloudLabelCount[label] = 0
        truePositiveCloudLabelCount[label] = 0
        falsePositiveCloudLabelCount[label] = 0

    truePositivOutlierCount = 0
    falsePositivOutlierCount = 0
    groundLabels = ["road", "sidewalk", "parking", "lane-marking"]
    additionalGroundLabels = ["other-ground", "terrain"]
    nonGroundLabels = ["bicycle", "moving-bicyclist", "motorcycle", "moving-motorcyclist", "person", "moving-person", "traffic-sign", "car", "moving-car",
                       "motorcyclist", "bicyclist", "truck", "moving-truck", "building", "fence", "trunk", "pole", "bus", "on-rails", "other-vehicle", "other-structure",
                       "other-object", "moving-on-rails", "moving-bus", "moving-other-vehicle"]

    rospy.Subscriber('/groundgrid/segmented_cloud', PointCloud2, callback_predicted_cloud)


    rospy.wait_for_service('/kitti_player/NextCloud')
    time.sleep(1) # wait a bit until the other nodes are launched
    try:
        nextCloud = rospy.ServiceProxy('/kitti_player/NextCloud', NextCloud)
        nextCloud()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    while not rospy.is_shutdown():
        rate.sleep()


def callback_predicted_cloud(cloud):
    global CFG, nonGroundPointLabelCount, semanticCloudLabelCount, truePositiveCloudLabelCount, falsePositiveCloudLabelCount, cloudCount, groundLabels, additionalGroundLabels
    global nextCloud

    for p in pc2.read_points(cloud, field_names = ("x", "y", "z", "intensity", "ring"), skip_nans=True):
        point = Point()
        point.x = p[0]
        point.y = p[1]
        point.z = p[2]
        intensity = p[3]
        label = p[4]
        labelstring = CFG["labels"][label]

        if intensity == 99: # predicted obstacle
            nonGroundPointLabelCount[labelstring] += 1
        elif intensity == 49: # predicted ground
            if labelstring in groundLabels:
                truePositiveCloudLabelCount[labelstring] += 1
            elif labelstring in additionalGroundLabels:
                truePositiveCloudLabelCount[labelstring] += 1
            else:
                falsePositiveCloudLabelCount[labelstring] += 1

        semanticCloudLabelCount[labelstring] += 1

    cloudCount += 1

    # print statistics from every 500 clouds
    if cloudCount % 500 == 0:
        print_statistics()

    
    try:
        nextCloud()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return


def print_statistics():
    global nonGroundPointLabelCount, semanticCloudLabelCount, truePositiveCloudLabelCount, falsePositiveCloudLabelCount, cloudCount, groundLabels, nonGroundLabels, CFG, additionalGroundLabels

    print("Stats")
    sequence = '{:02d}'.format(rospy.get_param('/kitti_player/sequence', '00'))
    print("Received " + str(cloudCount) + " point clouds. KITTI sequence " + sequence + ".")
    print("label\t\t\tnonground %\tground %\tnonground\ttotal")
    for label in CFG["labels"].values():
        nonGroundPoints = nonGroundPointLabelCount[label]
        totalPoints = semanticCloudLabelCount[label]
        if totalPoints == 0:
            continue
        if len(label) < 8:
            label += '\t'
        if len(label) < 16:
            label += '\t'
        print(label+'\t{:2.2%}\t\t{:2.2%}\t\t'.format(float(nonGroundPoints/totalPoints),1.0-float(nonGroundPoints/totalPoints))+str(nonGroundPoints)+"\t\t"+str(totalPoints))


    truePgroundsum = 0
    truePgroundsumadd = 0
    trueNgroundNoVeg = 0
    falsePgroundObssumNoVeg = 0
    falseNgroundNoVeg = 0
    gtgroundsum = 0
    gtgroundsumadd = 0
    gtnonGroundSumNoVeg = 0
    for label in groundLabels:
        truePgroundsum += truePositiveCloudLabelCount[label]
        gtgroundsum += semanticCloudLabelCount[label]
        falseNgroundNoVeg += nonGroundPointLabelCount[label]

    truePgroundsumadd = truePgroundsum
    gtgroundsumadd = gtgroundsum
    for label in additionalGroundLabels:
        truePgroundsumadd += truePositiveCloudLabelCount[label]
        gtgroundsumadd += semanticCloudLabelCount[label]
        falseNgroundNoVeg += nonGroundPointLabelCount[label]

    for label in nonGroundLabels:
        falsePgroundObssumNoVeg += falsePositiveCloudLabelCount[label]
        gtnonGroundSumNoVeg += semanticCloudLabelCount[label]
        trueNgroundNoVeg += nonGroundPointLabelCount[label]


    # precision recall calculation (without vegetation)
    truePositiveGround = truePgroundsumadd
    trueNegativeGround = trueNgroundNoVeg
    falsePositiveGround = falsePgroundObssumNoVeg
    falseNegativeGround = falseNgroundNoVeg
    precision = truePositiveGround/(falsePositiveGround+truePositiveGround)
    recall = truePositiveGround/(falseNegativeGround+truePositiveGround)
    f1 = (2*truePositiveGround)/(2*truePositiveGround+falsePositiveGround+falseNegativeGround)
    accuracy = (truePositiveGround+trueNegativeGround)/(truePositiveGround+trueNegativeGround+falsePositiveGround+falseNegativeGround)


    print('Precision\t\t{:2.2%}\t\t{:0}\t{:0}'.format(precision, truePositiveGround, falsePositiveGround))
    print('Recall\t\t\t{:2.2%}\t\t{:0}\t{:0}'.format(recall, truePositiveGround, falseNegativeGround))
    print('F1\t\t\t{:2.2%}\t\t{:0}\t\t{:0}'.format(f1, falsePositiveGround, falseNegativeGround))
    print('Accuracy\t\t{:2.2%}\t\t{:0}\t{:0}'.format(accuracy, (truePositiveGround+trueNegativeGround), (truePositiveGround+trueNegativeGround+falsePositiveGround+falseNegativeGround)))
    print('IoUg\t\t\t{:2.2%}'.format(truePositiveGround/(falsePositiveGround+gtgroundsumadd)))


if __name__ == '__main__':
    global nonGroundPointLabelCount, semanticCloudLabelCount, truePositiveCloudLabelCount, falsePositiveCloudLabelCount, cloudCount, groundLabels, nonGroundLabels, CFG, additionalGroundLabels
    global truePositivOutlierCount, falsePositivOutlierCount
    try:
        init()
    except rospy.ROSInterruptException:
        print('interrupted')
        print_statistics()
    pass
