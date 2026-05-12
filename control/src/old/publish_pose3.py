#!/usr/bin/env python
import rospy, tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu


class BaseLinkOdomPublisher:
    def __init__(self):
        rospy.init_node('base_link_slam_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.latest_imu = None
        self.rate = rospy.Rate(50)

        self.parent_frame = "slam_odom"
        self.child_frame = "slam_base_link"
        
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
                trans = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(0.1))

                odom_msg = Odometry()
                odom_msg.header.stamp = trans.header.stamp
                odom_msg.header.frame_id = self.parent_frame
                odom_msg.child_frame_id = self.child_frame

                # Pose from TF
                odom_msg.pose.pose.position = trans.transform.translation
                odom_msg.pose.pose.orientation = trans.transform.rotation

                # Twist from IMU (only angular velocity here)
                if self.latest_imu:
                    odom_msg.twist.twist.angular = self.latest_imu.angular_velocity
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
