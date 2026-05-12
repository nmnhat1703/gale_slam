#!/usr/bin/env python
import rospy, tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped


class BaseLinkOdomPublisher:
    def __init__(self):
        rospy.init_node('base_link_slam_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        self.rate = rospy.Rate(30)
        
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

                pose_msg = PoseStamped()
                pose_msg.header.stamp = now
                pose_msg.header.frame_id = "odom"

                # Pose from TF
                pose_msg.pose.position = trans.transform.translation
                pose_msg.pose.orientation = trans.transform.rotation

                self.pose_pub.publish(pose_msg)

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                pass

            self.rate.sleep()

if __name__ == '__main__':
    try:
        BaseLinkOdomPublisher()
    except rospy.ROSInterruptException:
        pass
