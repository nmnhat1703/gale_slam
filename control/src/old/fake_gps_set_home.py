#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandHome


class FakeGPSHomeSetter:
    def __init__(self):
        self.pub = rospy.Publisher('/mavros/fake_gps/vision', PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.vision_cb)
        self.latest_pose = None

        # Wait for service
        rospy.wait_for_service('/mavros/cmd/set_home')
        self.set_home_srv = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

        # Set home after slight delay to ensure GPS data is flowing
        rospy.Timer(rospy.Duration(3), self.set_home_once, oneshot=True)

    def vision_cb(self, msg):
        self.latest_pose = msg
        self.pub.publish(msg)

    def set_home_once(self, event):
        try:
            rospy.loginfo("Attempting to set home using current GPS position...")
            resp = self.set_home_srv(current_gps=True, yaw=0.0,
                                     latitude=0.0, longitude=0.0, altitude=0.0)
            rospy.loginfo("Set home response: success=%s, result=%d", resp.success, resp.result)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call set_home: %s", e)

if __name__ == '__main__':
    rospy.init_node('fake_gps_home_setter')
    FakeGPSHomeSetter()
    rospy.spin()
