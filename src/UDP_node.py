#!/usr/bin/env python3
import rospy 
import socket
import quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs.msg import Imu
import signal

num_msg_recieved= 0

UDP_IP = '192.168.12.21'
#UDP_IP = '127.0.0.53'
#UDP_PORT = 53
UDP_PORT = 18403
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

    

    pub_0 = rospy.Publisher('/adq/imu_UDP/data', Imu, queue_size=3)


    while not rospy.is_shutdown():
        # prevT = time.clock()
        # 
        num_msg_recieved += 1

        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        #print(data)
        rs= np.array(str(data).replace("\\n'","").split("/")[1:],dtype=float).reshape((3,3))
        r = R.from_matrix(rs)
        rvec=r.as_rotvec()
        #print(rs)
        q = quaternion.from_rotation_vector(rvec)
        #print(f"{r.as_euler('xyz', degrees=True)}")
        pub_0.publish(visulaize_imu(q))       

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