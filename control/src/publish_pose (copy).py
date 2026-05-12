#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

def make_cov(diag):
    cov = [0.0]*36
    idx = [0,7,14,21,28,35]
    for i,v in enumerate(diag):
        cov[idx[i]] = float(v)
    return cov

POSE_DIAG  = [0.01, 0.01, 0.02, 0.02, 0.02, 0.03]  # m^2, rad^2
TWIST_DIAG = [0.01, 0.01, 0.02, 0.10, 0.10, 0.30]  # (m/s)^2, (rad/s)^2

pub = None

def cb(msg: Odometry):
    out = msg
    out.pose.covariance  = make_cov(POSE_DIAG)
    out.twist.covariance = make_cov(TWIST_DIAG)
    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("odom_cov_override")
    in_topic  = rospy.get_param("~input",  "/odometry/filtered")
    out_topic = rospy.get_param("~output", "/mavros/odometry/out")

    pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)
    rospy.spin()
