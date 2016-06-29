#!/usr/bin/env python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

import rospy

def imu_callback(msg):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = msg.header.frame_id
    pose_msg.pose.orientation = msg.orientation

    pose_pub.publish(pose_msg)

rospy.init_node("imu_to_pose")

imu_sub = rospy.Subscriber("/johnny5/sensor/imu/raw", Imu, imu_callback)
pose_pub = rospy.Publisher("/johnny5/sensor/imu/pose", PoseStamped, queue_size=1000)

rospy.spin()