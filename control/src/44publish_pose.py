#!/usr/bin/env python3
import copy
import math
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Quaternion,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)

# -----------------------------
# Configurable covariances
# -----------------------------
POSE_DIAG = [
    0.01,  # X variance (m^2)   -> sigma 0.10 m
    0.01,  # Y
    0.04,  # Z                  -> sigma 0.20 m
    0.01,  # Roll (rad^2)
    0.01,  # Pitch
    0.03   # Yaw
]

# Linear velocity covariance diag (m/s)^2
VEL_DIAG = [
    0.05**2,  # vx variance
    0.05**2,  # vy variance
    0.10**2,  # vz variance
]

# Angular velocity covariance diag (rad/s)^2
ANGVEL_DIAG = [
    0.10**2,  # wx variance
    0.10**2,  # wy variance
    0.20**2,  # wz variance
]

def make_cov6_diag(x, y, z, roll, pitch, yaw):
    """Return 6x6 covariance flattened row-major with diagonal set."""
    cov = [0.0] * 36
    diag = [x, y, z, roll, pitch, yaw]
    idx = [0, 7, 14, 21, 28, 35]
    for i, v in enumerate(diag):
        cov[idx[i]] = float(v)
    return cov

POSE_COV36 = make_cov6_diag(*POSE_DIAG)

# For TwistWithCovarianceStamped:
# covariance is for [vx vy vz wx wy wz]
VEL_TWIST_COV36 = make_cov6_diag(
    VEL_DIAG[0], VEL_DIAG[1], VEL_DIAG[2],
    ANGVEL_DIAG[0], ANGVEL_DIAG[1], ANGVEL_DIAG[2]
)

# -----------------------------
# Frame conversion helpers
# -----------------------------
def flu_world_to_enu_xyz(x_f, y_l, z_u):
    # world vector in FLU axes -> world vector in ENU axes
    # x_enu = -y_left, y_enu = x_fwd, z_enu = z_up
    return (-y_l, x_f, z_u)

def quat_mul(a, b):
    # Quaternion multiplication (x,y,z,w) * (x,y,z,w)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz
    )

def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def quat_rotate(q, v):
    """
    Rotate vector v by quaternion q. q is (x,y,z,w), v is (x,y,z).
    Returns rotated (x,y,z).
    """
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    return quat_mul(quat_mul(q, qv), quat_conj(q))[:3]

def yaw_quat(yaw_rad):
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))

# +90deg about Z (used for your FLU-world -> ENU-world pose rotation)
Q_ROT_Z_90 = yaw_quat(math.pi / 2.0)

# -----------------------------
# ROS pubs
# -----------------------------
pub_pose = None
pub_twist_cov = None

def cb(msg: Odometry):
    # Get parameters
    twist_is_body_flu = rospy.get_param("~twist_is_body_flu", True)
    publish_angular = rospy.get_param("~publish_angular", False)

    # Copy so we don't mutate the original
    out = copy.deepcopy(msg)

    # --- Save ORIGINAL orientation for body->world velocity rotation ---
    q_orig = msg.pose.pose.orientation
    q_flu_world = (q_orig.x, q_orig.y, q_orig.z, q_orig.w)

    # -------------------------
    # 1) Pose: FLU-world -> ENU-world
    # -------------------------
    p = out.pose.pose.position
    p.x, p.y, p.z = flu_world_to_enu_xyz(p.x, p.y, p.z)

    q = out.pose.pose.orientation
    q_in = (q.x, q.y, q.z, q.w)
    q_out = quat_mul(Q_ROT_Z_90, q_in)
    out.pose.pose.orientation = Quaternion(*q_out)

    m = PoseWithCovarianceStamped()
    m.header.stamp = msg.header.stamp
    m.header.frame_id = "odom"           # ENU world frame
    m.pose.pose = out.pose.pose
    m.pose.covariance = POSE_COV36
    pub_pose.publish(m)
"""
    # -------------------------
    # 2) Twist: publish ENU world velocity
    # -------------------------
    v_in = msg.twist.twist.linear
    v_vec = (v_in.x, v_in.y, v_in.z)

    if twist_is_body_flu:
        # BODY-FLU -> WORLD-FLU using odom orientation
        v_world_flu = quat_rotate(q_flu_world, v_vec)
    else:
        # Already WORLD-FLU
        v_world_flu = v_vec

    # WORLD-FLU -> WORLD-ENU
    vx_enu, vy_enu, vz_enu = flu_world_to_enu_xyz(*v_world_flu)

    t = TwistWithCovarianceStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "odom"           # ENU world frame
    t.twist.twist.linear.x = vx_enu
    t.twist.twist.linear.y = vy_enu
    t.twist.twist.linear.z = vz_enu

    if publish_angular:
        # Transform angular velocity
        w_in = msg.twist.twist.angular
        w_vec = (w_in.x, w_in.y, w_in.z)
        
        if twist_is_body_flu:
            # BODY-FLU -> WORLD-FLU
            w_world_flu = quat_rotate(q_flu_world, w_vec)
        else:
            # Already WORLD-FLU
            w_world_flu = w_vec
        
        # WORLD-FLU -> WORLD-ENU
        wx_enu, wy_enu, wz_enu = flu_world_to_enu_xyz(*w_world_flu)
        
        t.twist.twist.angular.x = wx_enu
        t.twist.twist.angular.y = wy_enu
        t.twist.twist.angular.z = wz_enu
        t.twist.covariance = VEL_TWIST_COV36
    else:
        # Only provide linear covariance; set angular cov huge
        cov = [0.0] * 36
        cov[0] = VEL_DIAG[0]
        cov[7] = VEL_DIAG[1]
        cov[14] = VEL_DIAG[2]
        # Make angular part "unknown" by giving large variance
        big = 1e6
        cov[21] = big
        cov[28] = big
        cov[35] = big
        t.twist.covariance = cov
        
        # Set angular velocity to zero
        t.twist.twist.angular.x = 0.0
        t.twist.twist.angular.y = 0.0
        t.twist.twist.angular.z = 0.0

    pub_twist_cov.publish(t)
"""
if __name__ == "__main__":
    rospy.init_node("vio_to_mavros_pose_cov_and_speed_cov")

    pub_pose = rospy.Publisher("/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, queue_size=10)
    #pub_twist_cov = rospy.Publisher("/mavros/vision_speed/speed_twist_cov", TwistWithCovarianceStamped, queue_size=10)

    rospy.Subscriber("/odometry/filtered", Odometry, cb, queue_size=50)

    rospy.spin()