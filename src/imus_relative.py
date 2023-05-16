#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Author: Yang Chen
# @Date:   2020-11-08 23:25:16
# @Last Modified by:   robot-Yang
# @Last Modified time: 2022-12-11 19:36:51

# get transformation from one link to another, loop

import rospy

import math
import tf2_ros
import geometry_msgs.msg
# import turtlesim.srv
from scipy.spatial.transform import Rotation as Ro

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

if __name__ == '__main__':
    rospy.init_node('imus_lookup')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # pub_0 = rospy.Publisher('/imu2_relative_lookup', geometry_msgs.msg.Transform, queue_size=3)

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            trans1 = tfBuffer.lookup_transform("imu1_relative", "imu2_relative", rospy.Time())
            trans_1 = trans1.transform

            # euler = (Ro.from_quat([trans0.rotation.x,  trans0.rotation.y,  trans0.rotation.z,  trans0.rotation.w])).as_euler('zxy', degrees=True)
            # print('euler_imu2', euler)

            set_frame(trans_1.rotation, "imu2_relative_imu1")
           
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # pub_0.publish(trans0)


        rate.sleep()