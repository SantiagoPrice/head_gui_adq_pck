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

from geometry_msgs.msg import Wrench, WrenchStamped, Vector3, PoseStamped, Quaternion, Twist, TwistStamped, Pose2D
from sensor_msgs.msg import Imu , JointState
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Header

from termcolor import colored
import quaternion
from scipy.spatial.transform import Rotation as Ro

from sound_play.libsoundplay import SoundClient

imu_eular1 = [0.,0.,0.]
imu_eular2 = [0.,0.,0.]
imu_eular3 = [0.,0.,0.]

imu1_q = np.quaternion(0.,0.,0.,0.)
imu2_q = np.quaternion(0.,0.,0.,0.)
imu3_q = np.quaternion(0.,0.,0.,0.)

initial_imu1 = np.quaternion(0.,0.,0.,0.)
initial_imu2 = np.quaternion(0.,0.,0.,0.)

relative_imu1_q = np.quaternion(0.,0.,0.,0.)
relative_imu2_q = np.quaternion(0.,0.,0.,0.)

self_check_flag = 1
initializ_flag = 1

num_msg_recieved= 0

relative_imu_pose = Imu()
offset_imu_pose = np.quaternion(1.,0.,0.,0.) 


angles = [ "pitch" , "roll" , "yaw" , "pitch_ref" , "roll_ref" , "yaw_ref"]
pos = [0 for i in range(6)]
#head_angle = JointState(name=angles,position=pos)
ref_ypr = [0,0,0]
ref_flag = False

AU_PATH = "/home/santiago/catkin_ws/src/IMU_ADQ_pck/src/voice commands"
audiocommands = [["/forward bending.wav","/backward bending.wav"],["/left rotation.wav","/right rotation.wav"],["/left bending.wav","/right bending.wav"],"/neutral position.wav"]
audiofile=""

ERR_TOL=np.pi/180*2

# self check if the readings form FSR are OK, if not OK, better not start the control program, considering safety issue.
def self_check():
    global imu1_q, imu2_q, IMU1_q, IMU2_q
    IMU1_q = []
    IMU2_q = []
    # while imu1_q == np.quaternion(0.,0.,0.,0.) or imu2_q == np.quaternion(0.,0.,0.,0.):
    while imu2_q == np.quaternion(0.,0.,0.,0.):
        #print('wait for imu')
        pass

    print('pre check OK')

def callback_imu1(imu):
    global imu1_q # imu_eular1
    imu1_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    # imu1_e = (Ro.from_quat([imu1_q.x, imu1_q.y, imu1_q.z, imu1_q.w])).as_euler("zxy", degrees=True)
    # imu1_e = np.round(imu1_e, decimals = 1)

    e = tf.transformations.euler_from_quaternion((imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w))
    imu1_e = [i/math.pi*180 for i in e]
    imu1_e = np.round(imu1_e, decimals = 1)
    # print("imu1_e", imu1_e)
# 
def callback_imu2(imu):
    global imu2_q # imu_eular1
    # print ("imu =", imu)
    # imu2_q = imu.orientation
    imu2_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    # imu2_e = (Ro.from_quat([imu2_q.x, imu2_q.y, imu2_q.z, imu2_q.w])).as_euler("zxy", degrees=True)
    # imu2_e = np.round(imu2_e, decimals = 1)

    e = tf.transformations.euler_from_quaternion((imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w))
    imu2_e = [i/math.pi*180 for i in e]
    imu2_e = np.round(imu2_e, decimals = 1)
    # print("imu2_e", imu2_e)

def callback_imu3(imu):
    global imu3_q # imu_eular1
    # print ("imu =", imu)
    # imu3_q = imu.orientation
    imu3_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    # imu_eular1 = [i/math.pi*180 for i in e]

def callback_ref(imu):
    #global head_angle
    global ref_ypr , ref_flag, audiofile
    imu_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    #head_angle.position[3:] = yawPitchRoll(imu_q ,ls=True)
    
    ref_ypr = yawPitchRoll(imu_q ,ls=True)
    mots = [ang != 0 for ang in ref_ypr]
    if mots.count(True):
        mot_ind=mots.index(True)
        if ref_ypr[mot_ind] > 0:
            audiofile = AU_PATH + audiocommands[mot_ind][0]
        else:
            audiofile = AU_PATH + audiocommands[mot_ind][1]
    else:
        audiofile = AU_PATH + audiocommands[-1]
    ref_flag = True
    

def callback_offset(flag):
    global offset_imu_pose 
    a,b,aux= self_relative() 
    offset_imu_pose= 1/a*b
    print(offset_imu_pose)
    

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
    # print ("PYR orientation first IMU: {} degrees".format(Euang)) 
    # print ("Quaternion orientation first IMU:: {}".format(q1))   # depending on sensor, gyro data is outputted to g1, g2 or both
    #print ("G1: {} degree/s".format(np.arccos(q1.w)*2*180/np.pi))
    # print("{}".format('\n'*1))
    
    Euang2=yawPitchRoll(1/q2);
    # print ("PYR orientation second IMU: {}".format(Euang2)) 
    #print ("G2: {} degree/s".format(np.arccos(q2.w)*2*180/np.pi)) 
    # print ("Quaternion orientation second IMU: {}".format(q2))
    # print("{}".format('\n'*1))
    
    Euang21=yawPitchRoll(1/q2*q1)
    # Euang21 = list(Euang21)
    # Euang21 = [round(i, 3) for i in Euang21]
    Euang21 = np.round(Euang21, decimals = 1)
    # print ("PYR Orientation according to first sensor: {} degree/s".format(Euang21)) 
    # print ("PYR Orientation according to first sensor: degree/s") 
    # print (Euang21)
    # print("{.2f}".format(Euang21))
    # print("%.2f" % Euang21)
    # print("{}".format('\n'*2))

def visulaize_imu(q):
    # global relative_imu_pose  
    i = Imu()  
    i.header.stamp = rospy.Time.now()
    i.header.frame_id = 'imu'
    i.orientation = q
    return i
    # relative_imu_pose.angular_velocity = create_vector3(sci.groll, sci.gpitch, sci.gyaw)

def initial_pose():
    global imu1_q, imu2_q, IMU1_q, IMU2_q, initial_imu1, initial_imu2
    IMU1_q = np.array(()) #[]
    IMU2_q = np.array(()) #[]
    while len(IMU1_q)< 100 or len(IMU2_q)< 100:
        IMU1_q = np.append(IMU1_q, imu1_q)
        IMU2_q = np.append(IMU2_q, imu2_q)
    # print ("IMU1_q =", IMU1_q)
    initial_imu1 =  quaternion.mean_rotor_in_chordal_metric(IMU1_q)
    initial_imu2 =  quaternion.mean_rotor_in_chordal_metric(IMU2_q)
    # initial_imu1 = imu1_q
    # initial_imu2 = imu2_q
    # print ("initial_imu1 =", initial_imu1)

def self_relative():
    global imu1_q, imu2_q, initial_imu1, initial_imu2 #, relative_imu1_q, relative_imu2_q
    # relative_imu1_q  = imu1_q / initial_imu1
    # relative_imu1_q  = imu1_q * initial_imu1.inverse()
    relative_imu1_q  = 1/initial_imu1 * imu1_q
    relative_imu2_q  = 1/initial_imu2 * imu2_q
    imu2_relative_imu1_q = 1/ offset_imu_pose *1/relative_imu1_q *  relative_imu2_q 

    # imu2_relative_imu1_e= quaternion.as_rotation_vector(imu2_relative_imu1_q)
    #imu2_relative_imu1_e = (Ro.from_quat([imu2_relative_imu1_q.x, imu2_relative_imu1_q.y, imu2_relative_imu1_q.z, imu2_relative_imu1_q.w])).as_euler("zxy", degrees=True)

    # imu2_relative_imu1_e= quaternion.as_euler_angles(imu2_relative_imu1_q)
    # imu2_relative_imu1_e = [i/math.pi*180 for i in imu2_relative_imu1_e]
    #imu2_relative_imu1_e = np.round(imu2_relative_imu1_e, decimals = 1)
    #print("imu2_relative_imu1_e", imu2_relative_imu1_e)
    return relative_imu1_q, relative_imu2_q, imu2_relative_imu1_q
    # e = tf.transformations.euler_from_quaternion((imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w))

def visualize_posit(q):
    global angles , pos ,ref_ypr
    head_state = JointState(name=angles,position=pos)
    head_state.header.stamp = rospy.Time.now()
    head_state.position[:3]= yawPitchRoll(q, ls = True)
    head_state.position[3:]= ref_ypr
    return head_state


# def imus_relative():
#     global relative_imu1_q, relative_imu2_q

def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    #print('-------------------------------------------------------- frequency = ', msg_fre)


def imu_measure_node():
    global self_check_flag, initializ_flag, num_msg_recieved, relative_imu_pose , ERR_TOL , ref_flag, audiofile#, relative_imu1_q, relative_imu2_q
    ########### Starting ROS Node ###########
    rospy.init_node('imu_relative_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    prev_orient = np.zeros(3)

    
    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    rospy.Subscriber("/imu1/imu/data", Imu, callback_imu1, queue_size=1) # imu1
    rospy.Subscriber("/imu2/imu/data", Imu, callback_imu2, queue_size=1) # imu2
    # rospy.Subscriber("/imu3/data", Imu, callback_imu3, queue_size=1) # imu2

    rospy.Subscriber('/adq/reference/data', Imu , callback_ref, queue_size=1)
    rospy.Subscriber('/adq/imu_rel/offset', Bool , callback_offset, queue_size=1)

    pub_0 = rospy.Publisher('/adq/imu_rel/imu1_realtive/data', Imu, queue_size=3)
    pub_1 = rospy.Publisher('/adq/imu_rel/imu2_realtive/data', Imu, queue_size=3)
    pub_00 = rospy.Publisher('/adq/imu_rel/imu1_initial/data', Imu, queue_size=3)
    pub_11 = rospy.Publisher('/adq/imu_rel/imu2_initial/data', Imu, queue_size=3)
    pub_2 = rospy.Publisher('/adq/imu_rel/imu2_realtive_imu1/data', Imu, queue_size=3)
    probe= rospy.Publisher('/adq/imu_rel/imu2_realtive_imu1/x', Float32, queue_size=3)
    probe2= rospy.Publisher('/adq/imu_rel/imu2_realtive_imu1/y', Float32, queue_size=3)
    probe3= rospy.Publisher('/adq/imu_rel/imu2_realtive_imu1/z', Float32, queue_size=3)
    #pub_h = rospy.Publisher('/adq/imu_rel/imu2_realtive_imu1/Rstate', JointState, queue_size=2000)
    pub_h = rospy.Publisher('/joint_states', JointState, queue_size=2000)
    cnt = 0
    flt_str=False
    pb_flag=True
    errs=0

    soundhandle = SoundClient()

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

        relative_imu1_q, relative_imu2_q, imu2_relative_imu1_q = self_relative()   # Function of control for Qolo
        #print(1/offset_imu_pose * imu2_relative_imu1_q )
        #imu2_relative_imu1_q= imu2_relative_imu1_q  / offset_imu_pose 
        #displayIMUs(relative_imu1_q, relative_imu2_q, imu2_relative_imu1_q)

        imu1_realtive = visulaize_imu(relative_imu1_q)
        imu2_realtive = visulaize_imu(relative_imu2_q)
        imu1_initial = visulaize_imu(initial_imu1)
        imu2_initial = visulaize_imu(initial_imu2)
        imu2_realtive_imu1 = visulaize_imu(imu2_relative_imu1_q)

        #visualize_posit(imu2_relative_imu1_q)
        head_angle = visualize_posit(imu2_relative_imu1_q)
        #print(head_angle.position)

        #nancond= not np.isnan(np.array(head_angle.position)).any()
        #zerocond = not (np.array(head_angle.position) < 0.5).all()
        

        if ref_flag:
            ref_flag = False
            soundhandle.playWave(audiofile,1)

        pub_h.publish(head_angle)

        
        """
        avg_disp= (np.array(head_angle.position[:3])-prev_orient).mean() 

        if avg_disp < ERR_TOL:

            if cnt < 5000: cnt+=1 
            else:        flt_str=True     
            
        else:
            if not flt_str: cnt=0               
            else:           pb_flag=True

        if pb_flag:

            print(yawPitchRoll(relative_imu1_q))
            #print(head_angle)
            pub_h.publish(head_angle)
            prev_orient = np.array(head_angle.position[:3])
        else:
            print("Noice was detected {}".format(errs))
            errs+=1
        pb_flag=True
        """
        #probe.publish(yawPitchRoll(imu2_relative_imu1_q, lsstd_msgs = True)[0])
        #probe.publish(head_angle.position[0])
        #probe2.publish(head_angle.position[1])
        #probe3.publish(head_angle.position[2])
        #pub_0.publish(imu1_realtive)
        #pub_1.publish(imu2_realtive)
        #pub_00.publish(imu1_initial)
        #pub_11.publish(imu2_initial)
        #pub_2.publish(imu2_realtive_imu1)
        

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