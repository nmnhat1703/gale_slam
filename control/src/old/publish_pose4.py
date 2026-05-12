#!/usr/bin/env python
import rospy, tf, tf2_ros, math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu


class BaseLinkOdomPublisher:
    def __init__(self):
        rospy.init_node('base_link_odom_imu_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.latest_imu = None
        self.rate = rospy.Rate(50)
        
        rospy.loginfo("Waiting for odom → base_link transform to become available...")
        try:
            self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(2.0))
            rospy.loginfo("TF odom → base_link is now available.")
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logerr("TF odom → base_link not found. Make sure LIO-SAM is running.")
            rospy.signal_shutdown("Missing TF: odom → base_link")
            return

        self.run()

    def imu_callback(self, msg):
        self.latest_imu = msg

    def run(self):
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                trans = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(0.1))

                odom_msg = Odometry()
                odom_msg.header.stamp = now
                odom_msg.header.frame_id = "odom_ned"
                odom_msg.child_frame_id = "base_link_frd"

                # Convert FLU (ROS default) to FRD (used in MAVROS)
                x_rot = trans.transform.translation.y
                y_rot = trans.transform.translation.x
                z_rot = trans.transform.translation.z

                odom_msg.pose.pose.position.x = x_rot
                odom_msg.pose.pose.position.y = y_rot
                odom_msg.pose.pose.position.z = -z_rot

                q_orig = (
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w)

                q_yaw_correction = tf.transformations.quaternion_from_euler(0, 0, 0)

                # Then rotate 180° around X (FLU → FRD)
                q_flip = tf.transformations.quaternion_from_euler(0, 0, 0)

                # Apply combined rotation: flip * yaw_correction * original
                q_tmp = tf.transformations.quaternion_multiply(q_yaw_correction, q_orig)
                q_final = tf.transformations.quaternion_multiply(q_flip, q_tmp)

                odom_msg.pose.pose.orientation = Quaternion(*q_final)

                # Twist from IMU (only angular velocity here)
                if self.latest_imu:
                    odom_msg.twist.twist.angular.x = self.latest_imu.angular_velocity.x
                    odom_msg.twist.twist.angular.y = self.latest_imu.angular_velocity.y
                    odom_msg.twist.twist.angular.z = self.latest_imu.angular_velocity.z
                    odom_msg.twist.twist.linear.x = 0.0
                    odom_msg.twist.twist.linear.y = 0.0
                    odom_msg.twist.twist.linear.z = 0.0

                odom_msg.pose.covariance = [0.01] * 36
                odom_msg.twist.covariance = [0.01] * 36
                self.odom_pub.publish(odom_msg)

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                pass

            self.rate.sleep()

if __name__ == '__main__':
    try:
        BaseLinkOdomPublisher()
    except rospy.ROSInterruptException:
        pass
