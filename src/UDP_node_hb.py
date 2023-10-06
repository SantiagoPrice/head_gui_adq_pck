#!/usr/bin/env python3
import rospy 
import socket
import quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import signal

num_msg_recieved= 0

UDP_IP = '192.168.12.41'
#UDP_IP = '127.0.0.20'
UDP_PORT = 18403
q1 = quaternion.quaternion(1,0,0,0)
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

def get_YPR(q):
    ypr=yawPitchRoll(q)
    YPR=Vector3()
    YPR.x= ypr[0]
    YPR.y= ypr[1]
    YPR.z= ypr[2]
    return YPR

def imu_measure_node():
    global num_msg_recieved,sock ,q1
    ########### Starting ROS Node ###########
    rospy.init_node('imu_UDP_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    prev_orient = np.zeros(3)

    

    pub_1 = rospy.Publisher('/imu1/imu/data', Imu, queue_size=3)
    pub_2 = rospy.Publisher('/imu2/imu/data', Imu, queue_size=3)
    pub_1_rpy = rospy.Publisher('/imu1/imu/RPY', Vector3, queue_size=3)
    pub_2_rpy  = rospy.Publisher('/imu2/imu/RPY', Vector3, queue_size=3)
    pub_3_rpy  = rospy.Publisher('/imurel/imu/RPY', Vector3, queue_size=3)
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
            q90z=np.quaternion(-0.701,0,0.701)
                               
            pub_1.publish(visulaize_imu(q*q90z))
            pub_1_rpy.publish(get_YPR(1/q))
            q1=q
        elif ds2str[0][-1] == "h":
            pub_2.publish(visulaize_imu(q))
            pub_2_rpy.publish(get_YPR(q))
            pub_3_rpy.publish(get_YPR(1/q1*q))



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