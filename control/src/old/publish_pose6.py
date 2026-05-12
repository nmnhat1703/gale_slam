#!/usr/bin/env python
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist


class BaseLinkOdomPublisher:
    def __init__(self):
        rospy.init_node('base_link_slam_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)

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

    def run(self):
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                trans = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(0.1))

                odom_msg = Odometry()
                odom_msg.header.stamp = now
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"

                # Pose from TF
                odom_msg.pose.pose.position = trans.transform.translation
                odom_msg.pose.pose.orientation = trans.transform.rotation

                
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0
                odom_msg.twist.twist.linear.x = 0.0
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0

                odom_msg.pose.covariance = [
                                            0.05, 0, 0, 0, 0, 0,
                                            0, 0.05, 0, 0, 0, 0,
                                            0, 0, 0.05, 0, 0, 0,
                                            0, 0, 0, 0.02, 0, 0,
                                            0, 0, 0, 0, 0.02, 0,
                                            0, 0, 0, 0, 0, 0.02
                                            ]


                odom_msg.twist.covariance = [
                                            999, 0, 0, 0, 0, 0,
                                            0, 999, 0, 0, 0, 0,
                                            0, 0, 999, 0, 0, 0,
                                            0, 0, 0, 999, 0, 0,
                                            0, 0, 0, 0, 999, 0,
                                            0, 0, 0, 0, 0, 999
                                            ]

                self.odom_pub.publish(odom_msg)

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                pass

            self.rate.sleep()

if __name__ == '__main__':
    try:
        BaseLinkOdomPublisher()
    except rospy.ROSInterruptException:
        pass
