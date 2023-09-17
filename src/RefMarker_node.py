#!/usr/bin/env python3
import rospy 
import socket
import quaternion
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA , Float32
from visualization_msgs.msg import Marker
import signal
from matplotlib import colors
num_msg_recieved= 0

Mark = Marker()
imu1_q = np.quaternion(1.,0.,0.,0.)
imu_corr = np.quaternion(0.701,0.701,0.,0.)

def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    #print('-------------------------------------------------------- frequency = ', msg_fre)

def callback_imu1(imu):
    global imu1_q # imu_eular1
    imu1_q = 1/imu_corr *np.quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, -imu.orientation.z) *imu_corr 


def imu_measure_node():
    global num_msg_recieved,Mark ,imu1_q , imu_corr
    ########### Starting ROS Node ###########
    rospy.init_node('mark_node', anonymous=True)
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    prev_orient = np.zeros(3)

   

    sub_1 = rospy.Subscriber('/adq/reference/data' , Imu, callback_imu1, queue_size=1)
    pub_1=rospy.Publisher('adq/Marker/Ref',Marker,queue_size=3)
    while not rospy.is_shutdown():
        num_msg_recieved += 1
        Mark.header.frame_id="myFrame"
        Mark.header.stamp=rospy.Time.now()

        Mark.ns="RMark"
        Mark.id=0

        Mark.type = Marker.MESH_RESOURCE
        Mark.mesh_resource="package://IMU_ADQ_pck/urdf/mesh/human-head-simplified.dae"

        Mark.action= Marker.ADD

        Mark.pose.position.x=0
        Mark.pose.position.y=0
        Mark.pose.position.z=1

        Mark.pose.orientation.x=imu1_q.x
        Mark.pose.orientation.y=imu1_q.y
        Mark.pose.orientation.z=imu1_q.z
        Mark.pose.orientation.w=imu1_q.w

        Mark.scale.x = 0.01
        Mark.scale.y = 0.01
        Mark.scale.z = 0.01
        
        MrkRGBA = [float(c) for c in colors.to_rgba('green')]
        Mark.color=ColorRGBA(*MrkRGBA)
        Mark.color.a=0.5
        

        Mark.lifetime=rospy.Duration()

        pub_1.publish(Mark)



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