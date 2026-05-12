#!/usr/bin/env python
import rospy, tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped

class SLAMPosePublisher:
    def __init__(self):
        rospy.init_node('slam_pose_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_pub = rospy.Publisher('/slam_pose', PoseWithCovarianceStamped, queue_size=10)

        self.rate = rospy.Rate(30)
        
        rospy.loginfo("Waiting for odom → base_link transform to become available...")
        try:
            self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(2.0))
            rospy.loginfo("TF odom → base_link is now available.")
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logerr("TF odom → base_link not found. Make sure LIO-SAM is running.")
            rospy.signal_shutdown("Missing TF: odom → base_link")
            return

    def run(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(0.1))
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "odom"
                pose_msg.pose.pose.position = trans.transform.translation
                pose_msg.pose.covariance = [0.0] * 36  # Initialize covariance to zero
                pose_msg.pose.covariance[0] = 1e-5
                pose_msg.pose.covariance[7] = 1e-5
                pose_msg.pose.covariance[14] = 1e-5
                self.pose_pub.publish(pose_msg)

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                pass

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SLAMPosePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
