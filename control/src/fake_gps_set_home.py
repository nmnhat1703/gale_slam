
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odom_callback(msg):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()  # Use current time
    pose_msg.header.frame_id = "odom"  # Or "odom", depending on your frame
    pose_msg.pose = msg.pose.pose
    pub.publish(pose_msg)

rospy.init_node('slam_to_fake_gps')

# Publisher to FakeGPS plugin
pub = rospy.Publisher('/mavros/fake_gps/vision', PoseStamped, queue_size=10)

# Subscribe to SLAM odometry
rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

rospy.loginfo("Publishing SLAM pose to /mavros/fake_gps/vision...")
rospy.spin()
