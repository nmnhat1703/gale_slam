#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def callback(msg):
    q = msg.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    #rospy.loginfo("LIO-SAM Roll: {:.2f} deg".format(roll_deg))
    #rospy.loginfo("LIO-SAM Pitch: {:.2f} deg".format(pitch_deg))
    #rospy.loginfo("LIO-SAM Yaw: {:.2f} deg".format(yaw_deg))
    rospy.loginfo(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

def listener():
    rospy.init_node('yaw_monitor')
    rospy.Subscriber("/mavros/local_position/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
