#!/usr/bin/env python3
import rospy
import tf2_ros
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

class OdometryBridge:
    def __init__(self):
        rospy.init_node('odometry_bridge')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)

        # Frame mapping
        self.source_frames = {'parent': 'odom', 'child': 'base_link'}            # from LIO-SAM
        self.target_frames = {'parent': 'odom_ned', 'child': 'base_link_frd'}    # for PX4

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.source_frames['parent'],
                    self.source_frames['child'],
                    rospy.Time(0),
                    rospy.Duration(0.1)
                )

                pos = transform.transform.translation
                ori = transform.transform.rotation

                # Check for NaNs
                if any(math.isnan(v) for v in [pos.x, pos.y, pos.z]):
                    rospy.logwarn("Transform contains NaNs. Skipping...")
                    rate.sleep()
                    continue

                # Create Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = self.target_frames['parent']
                odom_msg.child_frame_id = self.target_frames['child']

                # Position: ENU (x=fwd, y=left, z=up) → NED (x=north, y=east, z=down)
                odom_msg.pose.pose.position.x = pos.y   # East → North
                odom_msg.pose.pose.position.y = pos.x   # North → East
                odom_msg.pose.pose.position.z = -pos.z  # Up → Down

                # Orientation: FLU → FRD (invert Y and Z of quaternion)
                odom_msg.pose.pose.orientation.x = ori.x
                odom_msg.pose.pose.orientation.y = -ori.y
                odom_msg.pose.pose.orientation.z = -ori.z
                odom_msg.pose.pose.orientation.w = ori.w

                # Twist (set to zero if not available)
                odom_msg.twist.twist.linear.x = 0.0
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0

                # Covariances
                odom_msg.pose.covariance = [0.01] * 36
                odom_msg.twist.covariance = [0.01] * 36

                # Publish
                self.pub.publish(odom_msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(1.0, f"TF lookup failed: {str(e)}")

            rate.sleep()


if __name__ == '__main__':
    try:
        bridge = OdometryBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
