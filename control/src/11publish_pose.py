#!/usr/bin/env python3
import copy
import math
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# -----------------------------
# Configurable covariances
# -----------------------------
POSE_DIAG = [
    0.01,  # X variance (m^2)
    0.01,  # Y
    0.04,  # Z
    0.01,  # Roll (rad^2)
    0.01,  # Pitch
    0.03   # Yaw
]

VEL_DIAG = [
    0.05**2,  # vx variance
    0.05**2,  # vy variance
    0.10**2,  # vz variance
]

ANGVEL_DIAG = [
    0.10**2,  # wx variance
    0.10**2,  # wy variance
    0.20**2,  # wz variance
]

def make_cov6_diag(x, y, z, roll, pitch, yaw):
    cov = [0.0] * 36
    diag = [x, y, z, roll, pitch, yaw]
    idx = [0, 7, 14, 21, 28, 35]
    for i, v in enumerate(diag):
        cov[idx[i]] = float(v)
    return cov

POSE_COV36 = make_cov6_diag(*POSE_DIAG)

# Twist covariance is [vx vy vz wx wy wz] in nav_msgs/Odometry
TWIST_COV36_FULL = make_cov6_diag(
    VEL_DIAG[0], VEL_DIAG[1], VEL_DIAG[2],
    ANGVEL_DIAG[0], ANGVEL_DIAG[1], ANGVEL_DIAG[2]
)

# -----------------------------
# Frame conversion helpers
# -----------------------------
def flu_world_to_enu_xyz(x_f, y_l, z_u):
    # World vector in FLU axes -> world vector in ENU axes
    # x_enu = -y_left, y_enu = x_fwd, z_enu = z_up
    return (-y_l, x_f, z_u)

def quat_mul(a, b):
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
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    return quat_mul(quat_mul(q, qv), quat_conj(q))[:3]

def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n > 1e-12:
        return (x/n, y/n, z/n, w/n)
    return (0.0, 0.0, 0.0, 1.0)

def yaw_quat(yaw_rad):
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))

# +90deg about Z (world FLU -> world ENU)
Q_ROT_Z_90 = yaw_quat(math.pi / 2.0)

pub_odom = None

def cb(msg: Odometry):
    twist_is_body_flu = rospy.get_param("~twist_is_body_flu", True)
    publish_angular = rospy.get_param("~publish_angular", False)

    # Original orientation (assumed child->parent, in original FLU world)
    q0 = msg.pose.pose.orientation
    q_flu_world = quat_normalize((q0.x, q0.y, q0.z, q0.w))

    # -------------------------
    # Output nav_msgs/Odometry for /mavros/odometry/out
    # Pose in parent frame (ENU world), Twist in child frame (base_link)
    # -------------------------
    out = Odometry()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = rospy.get_param("~parent_frame_id", "odom")      # treated as ENU world
    out.child_frame_id = rospy.get_param("~child_frame_id", "base_link")   # FLU body

    # ---- Pose: WORLD-FLU -> WORLD-ENU ----
    p = msg.pose.pose.position
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = \
        flu_world_to_enu_xyz(p.x, p.y, p.z)

    q_in = q_flu_world
    q_out = quat_mul(Q_ROT_Z_90, q_in)
    out.pose.pose.orientation = Quaternion(*q_out)

    out.pose.covariance = POSE_COV36

    # ---- Twist: MUST be in CHILD frame (base_link) for OdometryPlugin ----
    # Start by getting a body-frame linear velocity in FLU
    v = msg.twist.twist.linear
    v_vec = (v.x, v.y, v.z)

    if twist_is_body_flu:
        v_body_flu = v_vec
    else:
        # incoming is WORLD-FLU; convert to BODY-FLU using inverse rotation
        # v_body = R^-1 * v_world
        v_body_flu = quat_rotate(quat_conj(q_flu_world), v_vec)

    out.twist.twist.linear.x = v_body_flu[0]
    out.twist.twist.linear.y = v_body_flu[1]
    out.twist.twist.linear.z = v_body_flu[2]

    if publish_angular:
        w = msg.twist.twist.angular
        w_vec = (w.x, w.y, w.z)

        if twist_is_body_flu:
            w_body_flu = w_vec
        else:
            # incoming is WORLD-FLU; convert to BODY-FLU
            w_body_flu = quat_rotate(quat_conj(q_flu_world), w_vec)

        out.twist.twist.angular.x = w_body_flu[0]
        out.twist.twist.angular.y = w_body_flu[1]
        out.twist.twist.angular.z = w_body_flu[2]
        out.twist.covariance = TWIST_COV36_FULL
    else:
        # publish linear only; make angular "unknown"
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = 0.0

        cov = [0.0] * 36
        cov[0]  = VEL_DIAG[0]
        cov[7]  = VEL_DIAG[1]
        cov[14] = VEL_DIAG[2]
        big = 1e6
        cov[21] = big
        cov[28] = big
        cov[35] = big
        out.twist.covariance = cov

    pub_odom.publish(out)

if __name__ == "__main__":
    rospy.init_node("vio_to_mavros_odometry_out")

    pub_odom = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=10)
    rospy.Subscriber("/odometry/filtered", Odometry, cb, queue_size=50)

    rospy.spin()
