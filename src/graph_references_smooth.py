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

from sound_play.libsoundplay import SoundClient

from termcolor import colored
import quaternion





self_check_flag = 1
initializ_flag = 1
AU_PATH = "/home/santiago/catkin_ws/src/IMU_ADQ_pck/src/voice commands"
audiocommands = [["/forward bending.wav","/backward bending.wav"],["/left rotation.wav","/right rotation.wav"],["/left bending.wav","/right bending.wav"],"/neutral position.wav"]
audiofile=""


ndF=100

num_msg_recieved= 0

relative_imu_pose = Imu()
trig= False
trial = False
abort = False

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
    trig=flag.data

def callback_trial(flag):
    global trial #
    print ("flag was toggled")
    trial=flag.data


def callback_abort(flag):
    global abort #
    print ("flag was toggled")
    abort=flag.data




def yawPitchRoll(q, ls = False):
    """Function that converts quaternion to the yaw pitch roll representation
    This representation corresponds to a tait-bryan rotation of xyz-order.
    Input: 
        .q: quaternion
    Output:
        yaw , pitch roll in radians"""

    yaw = np.arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    pitch = np.arcsin(-2.0*(q.x*q.z - q.w*q.y));
    roll = np.arctan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    if ls:
        return [yaw , pitch , roll]
    else:
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
    global trig, trial , abort
    global self_check_flag, initializ_flag, num_msg_recieved, relative_imu_pose #, relative_imu1_q, relative_imu2_q
    ########### Starting ROS Node ###########
    rospy.init_node('graph_ref_node', anonymous=True)

    #rospy.Timer(rospy.Duration(1), count_msg)
    
    rate = rospy.Rate(ndF) #  1 [Hz]
    
    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    ref = rospy.Publisher('/adq/reference/data', Imu, queue_size=3)
    #trigger = rospy.Publisher('/reference/start', Bool , queue_size=1)
    rospy.Subscriber('/adq/reference/start', Bool , callback_start, queue_size=1)
    rospy.Subscriber('/adq/reference/trial', Bool , callback_trial, queue_size=1)
    rospy.Subscriber('/adq/reference/abort', Bool , callback_abort, queue_size=1)
    
    index=True

    sag_rng = (np.pi/6,1)
    cor_rng = (np.pi/6,1)
    rot_rng = (np.pi/4.5,1)

    exp_period = [5,3,6,3]
    #exp_period = [2,2]

    repetitions = 1
    activation = False
    pos_l=get_qpostures (sag_rng, rot_rng , cor_rng)

    pos_trial = get_qpostures(sag_rng, rot_rng , cor_rng)
    test_period = [2,3,4,3]
    postures= []

    soundhandle = SoundClient()
    period = [1,1,1,1]


    cr_cl=0
    o_reference_q=quaternion.quaternion(1,0,0,0)
    reference_q=quaternion.quaternion(1,0,0,0)
    int_ang=[]
    while not rospy.is_shutdown():

        num_msg_recieved += 1
        index= not index
        if len(postures)== 0:
            
            if trig:
                trig=False
                activation = False
                #trigger.publish(Bool(False))
                postures = list(pos_l).copy()*repetitions
                random.shuffle(postures)
                postures.append(quaternion.quaternion(1,0,0,0))
                soundhandle.playWave(AU_PATH + "/sequence start.wav" ,1)
                period = [exp_period[0]+exp_period[3],exp_period[1]+exp_period[2]]
                period_trans=[exp_period[1],exp_period[3]]
                cr_cl = test_period[3]*ndF
                
                

            if trial:
                trial=False
                activation=False
                #trigger.publish(Bool(False))
                postures = list(pos_trial).copy()*repetitions
                random.shuffle(postures)
                postures.append(quaternion.quaternion(1,0,0,0))
                soundhandle.playWave(AU_PATH + "/trial start.wav",1)
                period = [test_period[0]+test_period[3],test_period[1]+test_period[2]]
                period_trans=[test_period[1],test_period[3]]
                cr_cl = test_period[3]*ndF
                
        else:

            if  cr_cl == period[activation]*100:
                cr_cl=0
                activation= not activation
                o_reference_q=reference_q
                if activation: 
                    reference_q= postures.pop(0)

                    
                    
                else:
                    reference_q= quaternion.quaternion(1,0,0,0)

                ref_ypr = yawPitchRoll(reference_q ,ls=True)
                mots = [ang != 0 for ang in ref_ypr]
                if mots.count(True):
                    mot_ind=mots.index(True)
                    if ref_ypr[mot_ind] > 0:
                        audiofile = AU_PATH + audiocommands[mot_ind][0]
                    else:
                        audiofile = AU_PATH + audiocommands[mot_ind][1]
                else:
                    audiofile = AU_PATH + audiocommands[-1]    

                soundhandle.playWave(audiofile,1)

                Ttrans= period_trans[activation]

                disp_q=1/o_reference_q * reference_q
                rv = quaternion.as_rotation_vector(disp_q)
                ang_disp=np.linalg.norm(rv)
                rv/=ang_disp
                inst_ang_lin=np.linspace(0,ang_disp,Ttrans*ndF)
                #inst_ang_nonlin=((np.sin(inst_ang_lin/ang_disp*np.pi-np.pi/2)*abs(np.sin(inst_ang_lin/ang_disp*np.pi-np.pi/2)))+1)/2*ang_disp
                int_ang=list(inst_ang_lin)
                
                
                
                #print(postures)
            else:
                if int_ang:
                        inst_ang=int_ang.pop(0)
                        inst_rv=(rv*inst_ang).reshape(1,-1)

                        disp_q=quaternion.from_rotation_vector(inst_rv)[0]

                        reference = visulaize_imu(o_reference_q*disp_q)
                       
                        ref.publish(reference)

        if abort:
            abort=False
            soundhandle.playWave(AU_PATH + "/the sequence was terminated.wav",1)
            reference = visulaize_imu(quaternion.quaternion(1,0,0,0))
            ref.publish(reference)
            postures = []

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