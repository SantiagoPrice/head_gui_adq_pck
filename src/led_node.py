#! /usr/bin/env python3
import serial
import threading
import rospy 
import numpy as np
from std_msgs.msg import  Bool
import signal

trig= False

num_msg_recieved=0

def callback_start(flag):
    global trig #
    print ("flag was toggled")
    trig=flag.data

def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    #print('-------------------------------------------------------- frequency = ', msg_fre)


def imu_measure_node():
    global num_msg_recieved , trig

    ########### Starting ROS Node ###########
    rospy.init_node('F_sens_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]

    rospy.Subscriber('/adq/reference/start', Bool , callback_start, queue_size=1)
    

    ser = serial.Serial('/dev/ttyACM0', baudrate=9600, bytesize=8, timeout=None,parity="N",stopbits=1, rtscts=False)

    ser.write(bytes.fromhex('41'))
    
    while not rospy.is_shutdown():
        num_msg_recieved += 1

        if trig:
            trig=False
            ser.write(bytes.fromhex('41'))
        #rate.sleep()

    logger.exit()
# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)


if __name__ == '__main__':
    try:
        imu_measure_node()
    except rospy.ROSInterruptException:
        pass