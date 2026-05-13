#!/usr/bin/env python3
import copy
import math
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped

POSE_DIAG = [
    0.01,  # X variance (m^2)   -> sigma 0.10 m
    0.01,  # Y
    0.04,  # Z                  -> sigma 0.20 m
    0.01,  # Roll (rad^2)
    0.01,  # Pitch
    0.03   # Yaw
]

def make_cov(diag):
    cov = [0.0] * 36
    idx = [0, 7, 14, 21, 28, 35]
    for i, v in enumerate(diag):
        cov[idx[i]] = float(v)
    return cov

def flu_world_to_enu_xyz(x_f, y_l, z_u):
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

def yaw_quat(yaw_rad):
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))

Q_ROT_Z_90 = yaw_quat(math.pi / 2.0)

pub = None

def cb(msg: Odometry):
    out = copy.deepcopy(msg)

    # Position: FLU-world -> ENU-world
    p = out.pose.pose.position
    p.x, p.y, p.z = flu_world_to_enu_xyz(p.x, p.y, p.z)

    # Orientation: rotate world axes by +90deg about Z
    q = out.pose.pose.orientation
    q_in = (q.x, q.y, q.z, q.w)
    q_out = quat_mul(Q_ROT_Z_90, q_in)
    out.pose.pose.orientation = Quaternion(*q_out)

    m = PoseWithCovarianceStamped()
    m.header.stamp = msg.header.stamp      # keep sensor time
    m.header.frame_id = "odom"             # use a consistent fixed frame
    m.pose.pose = out.pose.pose
    m.pose.covariance = make_cov(POSE_DIAG)

    pub.publish(m)

if __name__ == "__main__":
    rospy.init_node("vio_to_mavros_pose_cov")

    in_topic = rospy.get_param("~input", "/Odometry")
    out_topic = rospy.get_param("~output", "/mavros/vision_pose/pose_cov")

    pub = rospy.Publisher(out_topic, PoseWithCovarianceStamped, queue_size=10)
    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)

    rospy.spin()