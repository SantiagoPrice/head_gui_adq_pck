#!/usr/bin/env python3
import rospy 
import socket
import quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs.msg import Imu
import signal

num_msg_recieved= 0

UDP_IP = '192.168.12.41'
#UDP_IP = '127.0.0.20'
UDP_PORT = 18403
#UDP_PORT = 44082
sock = socket.socket(socket.AF_INET, # Internet
                      socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
    

def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    #print('-------------------------------------------------------- frequency = ', msg_fre)



def visulaize_imu(q):
    # global relative_imu_pose  
    i = Imu()  
    i.header.stamp = rospy.Time.now()
    i.header.frame_id = 'imu'
    i.orientation = q
    return i


def imu_measure_node():
    global num_msg_recieved,sock
    ########### Starting ROS Node ###########
    rospy.init_node('imu_UDP_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    prev_orient = np.zeros(3)

    

    pub_1 = rospy.Publisher('/imu1/imu/data', Imu, queue_size=3)
    pub_2 = rospy.Publisher('/imu2/imu/data', Imu, queue_size=3)

    while not rospy.is_shutdown():
        # prevT = time.clock()
        # 
        num_msg_recieved += 1

        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        #print(data)
        ds2str=str(data).replace("\\n'","").split("/")

        
        
        rs= np.array(ds2str[2:],dtype=float).reshape((3,3))
            
        r1 = R.from_matrix(rs)
        
        
        rvec=r1.as_rotvec()
        #print(rs)
        q = quaternion.from_rotation_vector(rvec)
        #print(f"{r.as_euler('xyz', degrees=True)}")

        if ds2str[0][-1] == "t":
            pub_1.publish(visulaize_imu(q))
        elif ds2str[0][-1] == "h":
            pub_2.publish(visulaize_imu(q))
        



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