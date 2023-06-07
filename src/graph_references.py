#! /usr/bin/env python3
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
import random

from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion, Twist, TwistStamped, Pose2D
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Header

from termcolor import colored
import quaternion





self_check_flag = 1
initializ_flag = 1

num_msg_recieved= 0

relative_imu_pose = Imu()
trig= False

# self check if the readings form FSR are OK, if not OK, better not start the control program, considering safety issue.
def self_check():
    global imu1_q, imu2_q, IMU1_q, IMU2_q
    IMU1_q = []
    IMU2_q = []
    # while imu1_q == np.quaternion(0.,0.,0.,0.) or imu2_q == np.quaternion(0.,0.,0.,0.):
    while imu2_q == np.quaternion(0.,0.,0.,0.):
        print('wait for imu')

    print('pre check OK')

def callback_start(flag):
    global trig #
    print ("flag was toggled")
    # imu2_q = imu.orientation
    trig=flag.data

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

def displayIMUs(q1,q2,q3):
    """
    Returns
    -------
    None.
    """ 
    # q1_angrep= quaternion.as_rotation_vector(q1)
    # q2_angrep= quaternion.as_rotation_vector(q2)
    
    Euang=yawPitchRoll(1/q1);

    
    Euang2=yawPitchRoll(1/q2);
    
    Euang21=yawPitchRoll(1/q2*q1)

    Euang21 = np.round(Euang21, decimals = 1)


def visulaize_imu(q):
    # global relative_imu_pose  
    i = Imu()  
    i.header.stamp = rospy.Time.now()
    i.header.frame_id = 'imu'
    i.orientation = q
    return i
    # relative_imu_pose.angular_velocity = create_vector3(sci.groll, sci.gpitch, sci.gyaw)

def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    print('-------------------------------------------------------- frequency = ', msg_fre)

def get_qpostures (sag_rng, cor_rng, rot_rng):
    """ Function that returns the quaternion for each precribed posture:
    Input:
        .xxx_rng: (max ang, int_post)
            . max_ang maximum deflection  angle at the given plane
            . int_post: number of positions between maximum deflection and the straight pose
    Output:
        . qposes: quaterion list with each of the prescribed pose 
    """

    pos_lst= [sag_rng+(0,), cor_rng+(1,), rot_rng+(2,)]
    rVcts = np.zeros([1,3])

    for Mang , nPos , axis in pos_lst:
    
        axiss = np.zeros(((nPos+1)*2,3))       
        angs = np.linspace(-Mang,Mang,(nPos+1)*2)
        axiss[:,axis] = angs
        print(angs)
        rVcts = np.append(rVcts, axiss,axis=0)
    #print(rVcts)
    return quaternion.from_rotation_vector(rVcts[1:,:])

def ref_display_node():
    global trig
    global self_check_flag, initializ_flag, num_msg_recieved, relative_imu_pose #, relative_imu1_q, relative_imu2_q
    ########### Starting ROS Node ###########
    rospy.init_node('graph_ref_node', anonymous=True)

    #rospy.Timer(rospy.Duration(1), count_msg)
    
    rate = rospy.Rate(2) #  1 [Hz]
    
    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    ref = rospy.Publisher('/adq/reference/data', Imu, queue_size=3)
    #trigger = rospy.Publisher('/reference/start', Bool , queue_size=1)
    rospy.Subscriber('/adq/reference/start', Bool , callback_start, queue_size=1)
    
    index=True

    sag_rng = (np.pi/4.5,1)
    cor_rng = (np.pi/6,1)
    rot_rng = (np.pi/4.5,1)
    period = [15,7]
    repetitions = 2
    activation = True
    pos_l=get_qpostures (sag_rng, cor_rng, rot_rng)
    postures= []

    cr_cl=1
    while not rospy.is_shutdown():

        num_msg_recieved += 1
        index= not index
        if len(postures)== 0:
            if trig:
                trig=False
                #trigger.publish(Bool(False))
                postures = list(pos_l).copy()*repetitions
                random.shuffle(postures)
                postures.append(quaternion.quaternion(1,0,0,0))
                
        else:
            if  cr_cl == period[activation]:
                cr_cl=0
                if activation: 
                    reference_q= postures.pop(0)
                else:
                    reference_q= quaternion.quaternion(1,0,0,0)
                print(reference_q)
                reference = visulaize_imu(reference_q)
                ref.publish(reference)
                activation= not activation
                print(postures)
            cr_cl+=1
 

        rate.sleep()

    logger.exit()
# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
    try:
        ref_display_node()
    except rospy.ROSInterruptException:
        pass