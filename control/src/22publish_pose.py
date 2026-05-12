#!/usr/bin/env python3
import copy
import math
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import CompanionProcessStatus

ROTATE_TWIST_LINEAR = True
STATUS_RATE_HZ = 5.0
pub_pose = None

def quat_mul(a, b):
    """Quaternion multiply a*b, both as (x,y,z,w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz)

def quat_inv(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

Q_ROT_Z_90 = (0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4))

pub_odom = None
pub_status = None
last_status_time = None

def odom_cb(msg):
    global last_status_time
    out = Odometry()
    out.header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
    out.header.frame_id = "odom"        # ENU world
    out.child_frame_id = "base_link"    # FLU body

    # ---------- position (FLU world → ENU world) ----------
    x_f = msg.pose.pose.position.x
    y_l = msg.pose.pose.position.y
    z_u = msg.pose.pose.position.z

    out.pose.pose.position.x = -y_l
    out.pose.pose.position.y =  x_f
    out.pose.pose.position.z =  z_u

    # ---------- orientation (correct conjugation) ----------
    q_in = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )

    q_out = quat_mul(
        quat_mul(Q_ROT_Z_90, q_in),
        quat_inv(Q_ROT_Z_90)
    )

    out.pose.pose.orientation.x = q_out[0]
    out.pose.pose.orientation.y = q_out[1]
    out.pose.pose.orientation.z = q_out[2]
    out.pose.pose.orientation.w = q_out[3]

    # ---------- covariance (reasonable defaults) ----------
    out.pose.covariance = [
        0.01, 0,    0,    0,    0,    0,
        0,    0.01, 0,    0,    0,    0,
        0,    0,    0.02, 0,    0,    0,
        0,    0,    0,    0.02, 0,    0,
        0,    0,    0,    0,    0.02, 0,
        0,    0,    0,    0,    0,    0.03
    ]

    # ---------- twist (DEBUG SAFE MODE) ----------
    out.twist.twist.linear.x  = 0.0
    out.twist.twist.linear.y  = 0.0
    out.twist.twist.linear.z  = 0.0
    out.twist.twist.angular.x = 0.0
    out.twist.twist.angular.y = 0.0
    out.twist.twist.angular.z = 0.0

    out.twist.covariance = [0.1]*36

    pub_odom.publish(out)

    # 6) Companion status (VIO active)
    now = rospy.Time.now()
    if pub_status is not None:
        if last_status_time is None or (now - last_status_time).to_sec() >= (1.0 / STATUS_RATE_HZ):
            st = CompanionProcessStatus()
            st.header.stamp = now
            st.component = CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
            st.state = CompanionProcessStatus.MAV_STATE_ACTIVE
            pub_status.publish(st)
            last_status_time = now


if __name__ == "__main__":
    rospy.init_node("vio_to_px4_odom_enu")

    in_topic = rospy.get_param("~input", "/odometry/filtered")
    out_topic = rospy.get_param("~output", "/mavros/odometry/out")
    status_topic = rospy.get_param("~status_topic", "/mavros/companion_process/status")

    pub_odom = rospy.Publisher(out_topic, Odometry, queue_size=10)
    pub_status = rospy.Publisher(status_topic, CompanionProcessStatus, queue_size=1)

    rospy.Subscriber(in_topic, Odometry, odom_cb, queue_size=10)

    rospy.spin()

