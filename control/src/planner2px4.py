#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from quadrotor_msgs.msg import PositionCommand

class PX4PositionBridge:
    def __init__(self):
        rospy.init_node('planner2px4', anonymous=True)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.cmd_callback)

    def cmd_callback(self, msg):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom"  
        pose.pose.position.x = msg.position.x
        pose.pose.position.y = msg.position.y
        pose.pose.position.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw)
        pose.pose.orientation = Quaternion(*q)
 
        self.setpoint_pub.publish(pose)

if __name__ == '__main__':
    try:
        bridge = PX4PositionBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
