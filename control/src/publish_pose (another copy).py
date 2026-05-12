#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import CompanionProcessStatus
import copy

def make_cov(diag):
    cov = [0.0] * 36
    idx = [0, 7, 14, 21, 28, 35]
    for i, v in enumerate(diag):
        cov[idx[i]] = float(v)
    return cov

POSE_DIAG  = [0.01, 0.01, 0.02, 0.02, 0.02, 0.2]  # m^2, rad^2
TWIST_DIAG = [0.01, 0.01, 0.02, 0.10, 0.10, 0.30]  # (m/s)^2, (rad/s)^2

pub_odom = None
pub_status = None
publish_status = True

def cb(msg: Odometry):
    # Make a deep copy so we don't mutate the original message object
    out = copy.deepcopy(msg)

    # --- Covariance override (same as your original script) ---
    out.pose.covariance  = make_cov(POSE_DIAG)
    out.twist.covariance = make_cov(TWIST_DIAG)

    # --- PX4 expects body velocities in FRD (Forward-Right-Down) ---
    # ROS base_link is typically FLU (Forward-Left-Up)
    # Convert twist from FLU -> FRD by flipping Y and Z signs:
    out.twist.twist.linear.y  *= -1.0
    out.twist.twist.linear.z  *= -1.0
    out.twist.twist.angular.y *= -1.0
    out.twist.twist.angular.z *= -1.0

    pub_odom.publish(out)

    # --- Optional: Companion status (recommended by PX4 VIO guide) ---
    if publish_status and pub_status is not None:
        st = CompanionProcessStatus()
        st.header.stamp = rospy.Time.now()
        st.component = CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        st.state     = CompanionProcessStatus.MAV_STATE_ACTIVE
        pub_status.publish(st)

if __name__ == "__main__":
    rospy.init_node("odom_to_px4_vio")

    in_topic  = rospy.get_param("~input",  "/odometry/filtered")
    out_topic = rospy.get_param("~output", "/mavros/odometry/out")
    publish_status = rospy.get_param("~publish_status", True)
    status_topic = rospy.get_param("~status_topic", "/mavros/companion_process/status")

    pub_odom = rospy.Publisher(out_topic, Odometry, queue_size=10)
    if publish_status:
        pub_status = rospy.Publisher(status_topic, CompanionProcessStatus, queue_size=1)

    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)
    rospy.spin()
