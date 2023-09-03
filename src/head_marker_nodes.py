#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
import signal

IMU_ref=Marker()
IMU_ref.type = IMU_ref.MESH_RESOURCE
IMU_ref.mesh_resource = "//IMU_ADQ_pck/urdf/mesh/human-head-simplified.stl"  # Replace with the actual path to your mesh file
IMU_ref.scale.x = 1.0  # Adjust the scale as needed
IMU_ref.scale.y = 1.0
IMU_ref.scale.z = 1.0
IMU_ref.color.r = 1.0
IMU_ref.color.g = 0.0
IMU_ref.color.b = 0.0
IMU_ref.color.a = 1.0

IMU_mrk=Marker()
IMU_mrk.type = IMU_mrk.MESH_RESOURCE
IMU_mrk.mesh_resource = "//IMU_ADQ_pck/urdf/mesh/human-head-simplified.stl"  # Replace with the actual path to your mesh file
IMU_mrk.scale.x = 1.0  # Adjust the scale as needed
IMU_mrk.scale.y = 1.0
IMU_mrk.scale.z = 1.0
IMU_mrk.color.r = 1.0
IMU_mrk.color.g = 0.0
IMU_mrk.color.b = 0.0
IMU_mrk.color.a = 1.0

num_msg_recieved=0

def imu_callback(msg):
    # Create visualization markers for IMU using a custom mesh
    global IMU_mrk
    IMU_mrk.header = msg.header
    IMU_mrk.pose.orientation = msg.orientation

def ref_callback(msg):
    # Create visualization markers for IMU using a custom mesh
    global IMU_ref
    IMU_ref.header = msg.header
    IMU_ref.pose.orientation  = msg.orientation
    
def count_msg(event):
    global num_msg_recieved
    msg_fre = num_msg_recieved / 1
    num_msg_recieved = 0
    #print('-------------------------------------------------------- frequency = ', msg_fre)

def imu_markers_node():
    global num_msg_recieved
    rospy.init_node('imu_visualization_node')
    rospy.Timer(rospy.Duration(1), count_msg)
    rate = rospy.Rate(100) #  100 [Hz]
    rospy.Subscriber('/adq/imu_rel/imu2_realtive_imu1/data', Imu, imu_callback, queue_size=1)
    rospy.Subscriber('/adq/reference/data', Imu , ref_callback, queue_size=1)

    marker_pub = rospy.Publisher('/imu_marker_array', MarkerArray, queue_size=10)
    # Publish the marker
    while not rospy.is_shutdown():
        num_msg_recieved += 1
        marker_array = MarkerArray()
        #marker_array.markers.append(IMU_ref)
        marker_array.markers.append(IMU_mrk)
        marker_pub.publish(marker_array)
        rate.sleep()

# for interruption
signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)


if __name__ == '__main__':
    try:
        imu_markers_node()
    except rospy.ROSInterruptException:
        pass
    