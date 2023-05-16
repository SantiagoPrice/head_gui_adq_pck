#! /usr/bin/env python3
'''
This program is for measuring imu relative to whill through camera T265, focusing on the z axis and y axis.
The imu and whill don't have to be under the same physical poses before starting this program,
but, their initial pysical x axis should be parallel and horizontal.
'''
import time
import math
import os
from itertools import groupby
import numpy as np
import heapq
import logging
from logging import handlers
import signal
import datetime
import rospy
import tf
# import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion, Twist, TwistStamped, Pose2D
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Header

from termcolor import colored
import quaternion
# from scipy.spatial.transform import Rotation as Ro

imu_eular1 = [0.,0.,0.]
imu_eular2 = [0.,0.,0.]
imu_eular3 = [0.,0.,0.]

T265_q = np.quaternion(0.,0.,0.,0.)
imu1_q = np.quaternion(0.,0.,0.,0.)
imu3_q = np.quaternion(0.,0.,0.,0.)
whill_q = np.quaternion(0.,0.,0.,0.)

initial_T265 = np.quaternion(0.,0.,0.,0.)
initial_imu1 = np.quaternion(0.,0.,0.,0.)

relative_T265_q = np.quaternion(0.,0.,0.,0.)
relative_imu1_q = np.quaternion(0.,0.,0.,0.)

self_check_flag = 1
initializ_flag = 1

num_msg_recieved= 0

relative_imu_pose = Imu()

# self check if the readings form FSR are OK, if not OK, better not start the control program, considering safety issue.
def self_check():
    global T265_q, imu1_q, TT265_q, IMU1_q
    TT265_q = []
    IMU1_q = []
    # while T265_q == np.quaternion(0.,0.,0.,0.) or imu1_q == np.quaternion(0.,0.,0.,0.):
    while imu1_q == np.quaternion(0.,0.,0.,0.):
        print('wait for imu')

    print('pre check OK')

def callback_T265(cameraPose):
    global T265_q # imu_eular1
    T265_q = np.quaternion(cameraPose.pose.pose.orientation.w, cameraPose.pose.pose.orientation.x, cameraPose.pose.pose.orientation.y, cameraPose.pose.pose.orientation.z)
    T265_e = ToDegree(T265_q, "sxyz")
    # print("T265_e", np.round(T265_e, decimals = 1))

    whill_pose(T265_q)

def whill_pose(T265_q):
    global whill_q
    # whill_relative_T265_e = [0, np.pi, 0]
    # whill_relative_T265 =  quaternion.from_euler_angles(whill_relative_T265_e)
    whill_relative_T265 = np.quaternion(0., 0., 0., 1.)
    whill_relative_T265_e = ToDegree(whill_relative_T265, "syxz")
    # print("whill_relative_T265_e", whill_relative_T265_e)
    whill_q =  whill_relative_T265 * T265_q
 
def callback_imu1(imu):
    global imu1_q # imu_eular1
    imu1_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    imu1_e = ToDegree(imu1_q, "sxyz")
    # print("imu1_q", imu1_q)

def yawPitchRoll(q):
    """Function that converts quaternion to the yaw pitch roll representation
    Input: 
        .q: quaternion
    Output:
        yaw , pitch roll in radians"""

    yaw = np.arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    pitch = np.arcsin(-2.0*(q.x*q.z - q.w*q.y));
    roll = np.arctan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    return ((np.array((yaw , pitch , roll))+np.pi)%(2*np.pi)-np.pi) * 180/np.pi

def visulaize_imu(q):
    # global relative_imu_pose  
    i = Imu()  
    i.header.stamp = rospy.Time.now()
    i.header.frame_id = 'imu'
    i.orientation = q
    return i

def initial_pose():
    global whill_q, imu1_q, WHILL_q, IMU1_q, initial_whill, initial_imu1
    WHILL_q = np.array(()) #[]
    IMU1_q = np.array(()) #[]
    while len(WHILL_q)< 100 or len(IMU1_q)< 100:
        WHILL_q = np.append(WHILL_q, whill_q)
        IMU1_q = np.append(IMU1_q, imu1_q)
    initial_whill =  quaternion.mean_rotor_in_chordal_metric(WHILL_q)
    initial_imu1 =  quaternion.mean_rotor_in_chordal_metric(IMU1_q)

    initial_imu1_e = tf.transformations.euler_from_quaternion((initial_imu1.x, initial_imu1.y, initial_imu1.z, initial_imu1.w), "sxyz")
    initial_imu1_e = list(initial_imu1_e)
    initial_imu1_e[0] = 0.
    initial_imu1 = quaternion.from_euler_angles(initial_imu1_e)

def ToDegree(q, order): # order = "sxyz" or "syxz" ...
    e = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w), "sxyz")
    e = [i/math.pi*180 for i in e]
    e = np.round(e, decimals = 1)
    return e

def self_relative():
    global T265_q, imu1_q, initial_T265, initial_imu1, whill_q #, relative_T265_q, relative_imu1_q
    
    relative_whill_q = 1/initial_whill * whill_q
    relative_imu1_q  = 1/initial_imu1 * imu1_q
    
    relative_whill_e = ToDegree(relative_whill_q, "sxyz")
    # print("relative_whill_e_degree", np.round(relative_whill_e_degree, decimals = 1)) 

    relative_imu1_e = ToDegree(relative_imu1_q, "sxyz")
    # print("relative_imu1_e", np.round(relative_imu1_e, decimals = 1))

    imu1_relative_whill_q = 1/relative_whill_q * relative_imu1_q

    imu1_relative_whill_e = ToDegree(imu1_relative_whill_q, "sxyz")
    print("imu1_relative_whill_e", imu1_relative_whill_e)

    return relative_whill_q, relative_imu1_q, imu1_relative_whill_q

def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    print('-------------------------------------------------------- frequency = ', msg_fre)


def imu_measure_node():
    global self_check_flag, initializ_flag, num_msg_recieved, relative_imu_pose #, relative_T265_q, relative_imu1_q
    ########### Starting ROS Node ###########
    rospy.init_node('imu_relative_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    
    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    rospy.Subscriber("/imu1/data", Imu, callback_imu1, queue_size=1) # imu1
    rospy.Subscriber("/camera/odom/sample", Odometry, callback_T265, queue_size=1) # T265

    # pub_0 = rospy.Publisher('/T265_realtive/data', Imu, queue_size=3)
    # pub_1 = rospy.Publisher('/imu1_realtive/data', Imu, queue_size=3)
    # pub_00 = rospy.Publisher('/T265_initial/data', Imu, queue_size=3)
    # pub_11 = rospy.Publisher('/imu1_initial/data', Imu, queue_size=3)
    # pub_2 = rospy.Publisher('/imu1_realtive_T265/data', Imu, queue_size=3)

    while not rospy.is_shutdown():
        # prevT = time.clock()
        # 
        num_msg_recieved += 1

        if self_check_flag == 1:
            self_check()
            self_check_flag = 0

        if initializ_flag == 1:
            initial_pose()
            initializ_flag = 0

        relative_whill_q, relative_imu1_q, imu1_relative_whill_q = self_relative()   # Function of control for Qolo
        
        # T265_realtive = visulaize_imu(relative_T265_q)
        # imu1_realtive = visulaize_imu(relative_imu1_q)
        # T265_initial = visulaize_imu(initial_T265)
        # imu1_initial = visulaize_imu(initial_imu1)
        # imu1_realtive_T265 = visulaize_imu(imu1_relative_T265_q)

        # pub_0.publish(T265_realtive)
        # pub_1.publish(imu1_realtive)
        # pub_00.publish(T265_initial)
        # pub_11.publish(imu1_initial)
        # pub_2.publish(imu1_realtive_T265)

        rate.sleep()

    logger.exit()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
    try:
        imu_measure_node()
    except rospy.ROSInterruptException:
        pass