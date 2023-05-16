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
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Header

from termcolor import colored
import quaternion

from numpy.linalg import inv
from scipy.spatial.transform import Rotation as Ro

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

# self check if the readings form FSR are OK, if not OK, better not start the control program, considering safety issue.
def self_check():
    global imu1_q, imu2_q, IMU1_q, IMU2_q
    IMU1_q = []
    IMU2_q = []
    # while imu1_q == np.quaternion(0.,0.,0.,0.) or imu2_q == np.quaternion(0.,0.,0.,0.):
    while imu2_q == np.quaternion(0.,0.,0.,0.):
        print('wait for imu')

    print('pre check OK')

def callback_imu1(imu):
    global imu1_q # imu_eular1
    imu1_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    imu1_e= quaternion.as_rotation_vector(imu1_q)
    # 
    # imu1_e = tf.transformations.euler_from_quaternion((imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w))
    # imu1_e =  [round(i/math.pi*180,) for i in imu1_e] # ((imu1_e+np.pi)%(2*np.pi)-np.pi) * 180/np.pi
    # print(imu1_e)

def callback_imu2(imu):
    global imu2_q # imu_eular1
    # print ("imu =", imu)
    # imu2_q = imu.orientation
    imu2_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    imu2_e= quaternion.as_rotation_vector(imu2_q)
    imu2_e =  [round(i/math.pi*180,) for i in imu2_e]
    # print(imu2_e)
    # 

def callback_imu3(imu):
    global imu3_q # imu_eular1
    # print ("imu =", imu)
    # imu3_q = imu.orientation
    imu3_q = np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
    # imu_eular1 = [i/math.pi*180 for i in e]

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
    # print ("PYR orientation first IMU: {} degrees".format(Euang)) 
    # q1_angrep = ((q1_angrep+np.pi)%(2*np.pi)-np.pi) * 180/np.pi
    # print(q1_angrep)
    # print ("Quaternion orientation first IMU:: {}".format(q1))   # depending on sensor, gyro data is outputted to g1, g2 or both
    #print ("G1: {} degree/s".format(np.arccos(q1.w)*2*180/np.pi))
    # print("{}".format('\n'*1))
    
    Euang2=yawPitchRoll(1/q2);
    # print ("PYR orientation second IMU: {}".format(Euang2)) 
    #print ("G2: {} degree/s".format(np.arccos(q2.w)*2*180/np.pi)) 
    # print ("Quaternion orientation second IMU: {}".format(q2))
    # print("{}".format('\n'*1))
    
    Euang21=yawPitchRoll(1/q2*q1);
    # print ("PYR Orientation according to first sensor: {} degree/s".format(Euang21)) 
    # print ("PYR Orientation according to first sensor: degree/s") 
    # print (Euang21)
    
    print("{}".format('\n'*1))

def visulaize_imu(q):
    # global relative_imu_pose  
    i = Imu()  
    i.header.stamp = rospy.Time.now()
    i.header.frame_id = 'imu'
    # i.orientation = q
    i.orientation.w = q.w
    i.orientation.x = q.x
    i.orientation.y = q.y
    i.orientation.z = q.z
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
    relative_imu1_q  = imu1_q / initial_imu1
    # relative_imu1_q  = imu1_q * initial_imu1.inverse()
    # relative_imu2_q  = imu2_q / initial_imu2
    # relative_imu2_q  =  imu2_q / initial_imu2
    # relative_imu1_q  = initial_imu1 / imu1_q
    # imu1_m = quaternion.as_rotation_matrix(imu1_q)
    # imu2_m = quaternion.as_rotation_matrix(imu2_q)
    imu2_m = (Ro.from_quat([imu2_q.x, imu2_q.y, imu2_q.z, imu2_q.w])).as_matrix()
    translation = [0,0,0]
    imu2_m = np.insert(imu2_m, 3, np.array(translation), axis=1)
    imu2_m = np.insert(imu2_m, 3, np.array([0.,0.,0.,1.]), axis=0)
    # initial_imu1_m = quaternion.as_rotation_matrix(initial_imu1)
    # initial_imu2_m = quaternion.as_rotation_matrix(initial_imu2)
    initial_imu2_m = (Ro.from_quat([initial_imu2.x, initial_imu2.y, initial_imu2.z, initial_imu2.w])).as_matrix()
    translation = [0,0,0]
    initial_imu2_m = np.insert(initial_imu2_m, 3, np.array(translation), axis=1)
    initial_imu2_m = np.insert(initial_imu2_m, 3, np.array([0.,0.,0.,1.]), axis=0)
    # 
    # relative_imu1_m = imu1_m.dot(inv(initial_imu1_m)) 
    # relative_imu1_m = initial_imu1_m.dot(inv(imu1_m))
    # relative_imu2_m = imu2_m.dot(inv(initial_imu2_m)) 
    relative_imu2_m = inv(initial_imu2_m).dot(imu2_m) 
    relative_imu2_m = relative_imu2_m[0:3,0:3]
    # relative_imu2_m = imu2_m.dot(initial_imu2_m.T) 

    # relative_imu1_q = quaternion.from_rotation_matrix(relative_imu1_m, nonorthogonal=True)
    # relative_imu1_e= quaternion.as_rotation_vector(relative_imu1_q)
    # relative_imu2_q = quaternion.from_rotation_matrix(relative_imu2_m, nonorthogonal=True)
    relative_imu2_q_ = (Ro.from_matrix(relative_imu2_m)).as_quat()
    print ("relative_imu2_q_", relative_imu2_q_)
    relative_imu2_q = np.quaternion(relative_imu2_q_[3], relative_imu2_q_[0], relative_imu2_q_[1], relative_imu2_q_[2])
    # relative_imu2_q = np.quaternion(relative_imu2_q_[0], relative_imu2_q_[1], relative_imu2_q_[2], relative_imu2_q_[3])

    relative_imu2_e= quaternion.as_rotation_vector(relative_imu2_q)
    relative_imu2_e =  [round(i/math.pi*180, 2) for i in relative_imu2_e]
    print(relative_imu2_e)

    imu2_relative_imu1_q = relative_imu2_q / relative_imu1_q
    return relative_imu1_q, relative_imu2_q, imu2_relative_imu1_q

def set_frame(q, child_frame):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "imu"
    t.child_frame_id = child_frame
    t.transform.translation.x = 0 # red axis, front_back direction
    t.transform.translation.y = 0
    t.transform.translation.z = 0 # blue axis, vertical one, need calibration

    t.transform.rotation.x = q.x
    t.transform.rotation.y = q.y
    t.transform.rotation.z = q.z
    t.transform.rotation.w = q.w

    br.sendTransform(t)

def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    print('-------------------------------------------------------- frequency = ', msg_fre)


def imu_measure_node():
    global self_check_flag, initializ_flag, num_msg_recieved, relative_imu_pose, initial_imu1, initial_imu2, imu1_q, imu2_q #, relative_imu1_q, relative_imu2_q
    ########### Starting ROS Node ###########
    rospy.init_node('imu_relative_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    
    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    rospy.Subscriber("/imu1/data", Imu, callback_imu1, queue_size=1) # imu1
    rospy.Subscriber("/imu2/data", Imu, callback_imu2, queue_size=1) # imu2
    rospy.Subscriber("/imu3/data", Imu, callback_imu3, queue_size=1) # imu2

    pub_0 = rospy.Publisher('/imu1_realtive/data', Imu, queue_size=3)
    pub_1 = rospy.Publisher('/imu2_realtive/data', Imu, queue_size=3)
    pub_00 = rospy.Publisher('/imu1_initial/data', Imu, queue_size=3)
    pub_11 = rospy.Publisher('/imu2_initial/data', Imu, queue_size=3)
    pub_2 = rospy.Publisher('/imu2_realtive_imu1/data', Imu, queue_size=3)

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
        
        displayIMUs(relative_imu1_q, relative_imu2_q, imu2_relative_imu1_q)

        imu1_realtive = visulaize_imu(relative_imu1_q)
        imu2_realtive = visulaize_imu(relative_imu2_q)
        imu1_initial = visulaize_imu(initial_imu1)
        imu2_initial = visulaize_imu(initial_imu2)
        imu2_realtive_imu1 = visulaize_imu(imu2_relative_imu1_q)

        pub_0.publish(imu1_realtive)
        pub_1.publish(imu2_realtive)
        pub_00.publish(imu1_initial)
        pub_11.publish(imu2_initial)
        pub_2.publish(imu2_realtive_imu1)

        set_frame(initial_imu1, "imu1_initial")
        set_frame(imu1_q, "imu1_q")
        set_frame(initial_imu2, "imu2_initial")
        set_frame(imu2_q, "imu2_q")
        
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