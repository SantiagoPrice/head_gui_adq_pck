#! /usr/bin/env python3
import serial
import threading
import rospy 
import numpy as np
from geometry_msgs.msg import Vector3
import signal

MEAS_AVG=70
num_msg_recieved=0
filename="/home/santiago/catkin_ws/src/IMU_ADQ_pck/src/participant_record/force_elia"
angle="30s"
def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    #print('-------------------------------------------------------- frequency = ', msg_fre)


def imu_measure_node():
    global num_msg_recieved

    ########### Starting ROS Node ###########
    rospy.init_node('F_sens_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    pub = rospy.Publisher('/Fsen/data', Vector3, queue_size=3)
    FXYZ=Vector3()

    ser = serial.Serial('/dev/ttyUSB1', baudrate=921600, bytesize=8, timeout=None,parity="N",stopbits=1, rtscts=1)
    # Set up
    #A2
    ser.write(bytes.fromhex('41'))
    ser.write(bytes.fromhex('32'))

    #F1
    ser.write(bytes.fromhex('46'))
    ser.write(bytes.fromhex('31'))

    #L0
    ser.write(bytes.fromhex('4C'))
    ser.write(bytes.fromhex('30'))
    #I
    ser.write(bytes.fromhex('49'))
    #HC
    ser.write(bytes.fromhex('48'))
    ser.write(bytes.fromhex('43'))

    # Adquisition 


    #R
    ser.write(bytes.fromhex('52'))
    ser.write(bytes.fromhex('53'))

    # First read
    FXavg=0
    FYavg=0
    FZavg=0
    i=0

    while not rospy.is_shutdown():
        # prevT = time.clock()
        # 
        num_msg_recieved += 1

        while True:
            st=ser.read(1)
            if st==b'\n':
                break
        seq=ser.read(1)
        FXr=ser.read(4)
        FYr=ser.read(4)
        FZr=ser.read(4)

        i+=1
        FX=int(str(FXr)[2:-1],16)
        FXavg+=(FX-16384)

        FY=int(str(FYr)[2:-1],16)
        FYavg+=(FY-16384)

        FZ=int(str(FZr)[2:-1],16)
        FZavg+=(FZ-16384)

        #print(f"seq is {int.from_bytes(seq, byteorder='little')}")
        if i>MEAS_AVG:
            i=0
            fxyz=np.array([float(FXavg),float(FYavg),float(FZavg)])/MEAS_AVG/240
            FXYZ.x=fxyz[0]
            FXYZ.y=fxyz[1]
            FXYZ.z=fxyz[2]
            with open(filename+angle+".txt","a") as f:
                f.write(",".join([str(fr) for fr in fxyz])+"\n")                      
            pub.publish(FXYZ)
            FXavg=0
            FYavg=0
            FZavg=0
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