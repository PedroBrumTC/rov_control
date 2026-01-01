#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, FluidPressure

class Pose:
    def __init__(self):

        # ===== Setup ROS =====

        rospy.init_node('Pose_node', anonymous=True)

        # Publisher
        self.state_pub = rospy.Publisher('state', Odometry, queue_size=10)

        # Subscriber
        self.imu_sub = rospy.Subscriber('imu_data', Imu, self.Imu_callback)
        self.pressure_sub = rospy.Subscriber('pressure_data', FluidPressure, self.pressure_callback)

        self.rate = rospy.Rate(10)  # Hz


        # ===== Parâmetros =====
        
        self.rho = rospy.get_param('~rho')
        self.g = rospy.get_param('~g')
        self.p0 = rospy.get_param('~patmospheric_pressure')
        self.depth_offset = rospy.get_param('~depth_offset')
        
        # Variáveis internas
        self.imu = Imu()
        self.depth = 0.0

 # ================= CALLBACKS =================


    def Imu_callback(self, msg):
        self.imu = msg
    def pressure_callback(self, msg):
         self.depth = (msg.fluid_pressure - self.p0) / (self.rho * self.g) + self.depth_offset



# ================= COMPUTE =================

    def compute_pose(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.orientation = self.imu.orientation
        odom.twist.twist.angular = self.imu.angular_velocity
        odom.pose.pose.position.z = self.depth

        return odom

# ================= LOOP =================

    def run(self):
        while not rospy.is_shutdown():
            pose = self.compute_pose()
            self.state_pub.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Pose()
        node.run()
    except rospy.ROSInterruptException:
        pass