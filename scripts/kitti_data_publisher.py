#!/usr/bin/env python3
import math
import rospy
import csv
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import tf2_ros
import tf
import numpy as np
import pandas as pd
import os
import sys, termios, fcntl
import time
import threading
from rosgraph_msgs.msg import Clock

from groundgrid.srv import NextCloud

DESIRED_RATE = 66  # Hz = 1/s


def init():
    global tfBuffer, dir, cloudnum, paused, stepping, timestamps, sim_clock, time_start, currentstamp, pausedTime
    paused = True
    stepping = False
    cloudnum = 0
    sequence = '{:02d}'.format(rospy.get_param('/kitti_player/sequence', 0))
    dir = rospy.get_param('/kitti_player/directory', '')
    if dir == '':
        rospy.logerr("no dataset directory given")
        return
    if dir[-1] != '/':
        dir += '/'
    dir = dir + 'sequences/' + sequence + '/'

    timestamps = pd.read_csv(f"{dir}times.txt", header=None,
                             dtype=float).squeeze("columns")

    rospy.init_node('kitti_data_publisher')
    rospy.loginfo('kitti_data_publisher initialized')
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    sim_clock = Clock()

    # set the simulation clock to the desired start time
    sim_clock.clock = rospy.Time.from_sec(
        rospy.get_param('/kitti_player/start', 0.0))

    sim_speed_multiplier = float(rospy.get_param('rate', 0.6))

    # should it start paused
    paused = rospy.get_param('/kitti_player/paused', True)

    # when the simulation is supposed to end
    end_timestamp = rospy.get_param("/kitti_player/end", np.inf)

    pausedTime = rospy.Duration(0)
    processPoses()

    pubTime = rospy.Publisher('clock', Clock, queue_size=10)
    pubCloud = rospy.Publisher('/kitti/cloud', PointCloud2, queue_size=10)
    pubPosition = rospy.Publisher('/localization/odometry/filtered_map',
                                  Odometry,
                                  queue_size=1)

    time_start = time.time() - sim_clock.clock.to_sec()
    currentstamp = -1

    input_thread = threading.Thread(target=terminal_input)
    input_thread.daemon = True
    input_thread.start()

    s_step = rospy.Service('/kitti_player/NextCloud', NextCloud, step)

    while not rospy.is_shutdown():
        start_loop = time.time()
        if not paused or stepping:
            new_time = sim_speed_multiplier * (
                time.time() - time_start) - pausedTime.to_sec()

            # end simulation prematurely based on configured end time
            if new_time > end_timestamp:
                return

            sim_clock.clock = rospy.Time.from_sec(new_time)
        else:
            pausedTime = rospy.Time.from_sec(
                sim_speed_multiplier *
                (time.time() - time_start)) - sim_clock.clock
        pubTime.publish(sim_clock)
        currentTime = sim_clock.clock
        (stampnum, nextstamp) = getNextTimeStamp(currentTime.to_sec())
        cloudnum = stampnum

        if (nextstamp > currentstamp):
            sendPosition(pubPosition, currentTime)
            sendCloud(pubCloud, currentTime)
            rospy.loginfo(
                f"published pointcloud and position --> {cloudnum=} with timestamp {nextstamp}"
            )
            currentstamp = nextstamp
            if stepping:
                stepping = False
                paused = True
        elif currentstamp > 0.0 and nextstamp == 0.0:  # sequence end
            return

        # if the simulation is running we want to spend at at least 1/RATE per iteration
        max_time_per_iteration = 1 / DESIRED_RATE


def sendCloud(pubCloud, currentTime):
    global cloudnum, dir
    cloudnumstring = f'{cloudnum:06}.bin'
    scan = np.fromfile(dir + "velodyne/" + cloudnumstring, dtype=np.float32)
    scan = scan.reshape((-1, 4))
    labels = readLabels(dir, cloudnum)
    tst2 = np.hstack((scan, labels[:, None].astype(dtype=np.uint16)))
 
    # cut cloud to labeled parts
    res = []
    sarray = np.core.records.fromarrays(tst2.T,
                                        dtype=[('x', 'float32'),
                                               ('y', 'float32'),
                                               ('z', 'float32'),
                                               ('remission', 'float32'),
                                               ('ring', 'uint16')])
    cloud_msg = PointCloud2()
    cloud_msg.header.stamp = currentTime
    cloud_msg.header.frame_id = 'kitti_base_link'
    cloud_msg.header.seq = cloudnum
    cloud_msg.height = sarray.shape[0]
    cloud_msg.width = 1
    cloud_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('ring', 16, PointField.UINT16, 1)
    ]
    cloud_msg.is_bigendian = False  # assumption
    cloud_msg.point_step = 18
    cloud_msg.row_step = cloud_msg.point_step
    cloud_msg.is_dense = True
    cloud_msg.data = sarray.tobytes()

    pubCloud.publish(cloud_msg)
    return


def readLabels(dir, cloudnum):
    cloudnumstring = f'{cloudnum:06}.label'
    labels = np.fromfile(dir + "labels/" + cloudnumstring, dtype=np.uint32)
    labels = labels.reshape((-1))
    labels = labels & 0xFFFF  # semantic label in lower half
    return labels


def processPoses():
    global dir, poses
    poses = []
    pose_file = os.path.join(dir, 'poses.txt')
    calibstring = "4.276802385584e-04 -9.999672484946e-01 -8.084491683471e-03 -1.198459927713e-02 -7.210626507497e-03 8.081198471645e-03 -9.999413164504e-01 -5.403984729748e-02 9.999738645903e-01 4.859485810390e-04 -7.206933692422e-03 -2.921968648686e-01"
    calib = np.fromstring(calibstring, dtype=float, sep=' ')
    calib = calib.reshape(3, 4)
    calib = np.vstack((calib, [0, 0, 0, 1]))
    calib_inv = np.linalg.inv(calib)
    with open(pose_file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            pose = np.fromstring(line, dtype=float, sep=' ')
            pose = pose.reshape(3, 4)
            pose = np.vstack((pose, [0, 0, 0, 1]))
            poses.append(np.matmul(calib_inv, np.matmul(pose, calib)))
    return


def sendPosition(pubPosition, currentTime):
    rospy.loginfo(f"sendPosition | {currentTime=}")
    global cloudnum, poses
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "kitti_map"
    rospy.loginfo(
        f"sendPosition | number of poses for cloudnum {cloudnum}: {len(poses[cloudnum])}"
    )
    odom.pose.pose.position.x = float(poses[cloudnum][0][3])
    odom.pose.pose.position.y = float(poses[cloudnum][1][3])
    odom.pose.pose.position.z = float(poses[cloudnum][2][3])
    rospy.loginfo(
        f"sendPosition | position: x={odom.pose.pose.position.x} y={odom.pose.pose.position.y} z={odom.pose.pose.position.z}"
    )

    R = poses[cloudnum]
    q = tf.transformations.quaternion_from_matrix(R)
    q_rot = tf.transformations.quaternion_from_euler(0, 0, 0)
    q_new = tf.transformations.quaternion_multiply(q_rot, q)
    odom.pose.pose.orientation.x = q_new[0]
    odom.pose.pose.orientation.y = q_new[1]
    odom.pose.pose.orientation.z = q_new[2]
    odom.pose.pose.orientation.w = q_new[3]
    br = tf.TransformBroadcaster()
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y,
                      odom.pose.pose.position.z),
                     tf.transformations.quaternion_from_euler(
                         tf.transformations.euler_from_quaternion(q_new)[0],
                         tf.transformations.euler_from_quaternion(q_new)[1],
                         tf.transformations.euler_from_quaternion(q_new)[2]),
                     currentTime, "kitti_base_link", "map")

    pubPosition.publish(odom)

    return


def getNextTimeStamp(time_passed):
    global timestamps

    # find first timestamp that is bigger then time passed
    timestamps_larger = timestamps[timestamps > time_passed]
    if timestamps_larger.empty:
        first_timestamp_larger_index = timestamps.shape[0]
    else:
        first_timestamp_larger_index = timestamps_larger.index[0]

    # return timestamp before that (starting to count at 0, thus no -1 for index)
    return (first_timestamp_larger_index - 1,
            timestamps.iloc[first_timestamp_larger_index - 1])


def pause(p):
    global paused
    paused = True
    return []


def unpause(p):
    global paused
    paused = False
    return []


def step(p):
    global paused, stepping
    paused = False
    stepping = True
    return []


def seek(p):
    global time_start, sim_clock, currentstamp, pausedTime

    sim_clock.clock = rospy.Time.from_sec(p.second)
    time_start = time.time() - sim_clock.clock.to_sec()
    currentstamp = -1
    pausedTime = rospy.Duration()


def terminal_input():
    global paused, stepping
    while True:
        input = getch()
        if input == ' ':
            paused = not paused
        elif input == 's' and not stepping:
            stepping = True

    return


# source: https://stackoverflow.com/a/7259460
def getch():
    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

    try:
        while 1:
            try:
                c = sys.stdin.read(1)
                break
            except IOError:
                pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
    return c


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
