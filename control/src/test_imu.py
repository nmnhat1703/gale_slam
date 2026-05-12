#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math


class IMUReader:
    def __init__(self):
        rospy.init_node('imu_rpy_reader')
        
        # Subscribe to IMU topic (change if different)
        self.imu_sub = rospy.Subscriber('/imu/data_reset', Imu, self.imu_callback, queue_size=30)
        
        # Initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        rospy.loginfo("IMU RPY Reader started. Waiting for IMU data...")


    def imu_callback(self, msg):
        # Extract quaternion from IMU message
        orientation_q = msg.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        
        # Convert quaternion to Euler angles (RPY)
        try:
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        except:
            rospy.logwarn("Invalid quaternion received")
            return
        
        # Convert radians to degrees
        self.roll = math.degrees(roll)
        self.pitch = math.degrees(pitch)
        self.yaw = math.degrees(yaw)
        
        # Print RPY values
        rospy.loginfo(f"Roll: {self.roll:.2f}°, Pitch: {self.pitch:.2f}°, Yaw: {self.yaw:.2f}°")
        #rospy.loginfo(quaternion)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        imu_reader = IMUReader()
        imu_reader.run()
    except rospy.ROSInterruptException:
        pass
