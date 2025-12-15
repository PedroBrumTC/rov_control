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
        
        # Densidade da água (kg/m³)
        self.rho = 1000.0
        self.g = 9.80665

        # Pressão atmosférica (Pa)
        self.p0 = 101325.0

        # Variáveis internas
        self.imu = Imu()
        self.depth = 0.0

 # ================= CALLBACKS =================


    def Imu_callback(self, msg):
        self.imu = msg
    def pressure_callback(self, msg):
         self.depth = (msg.fluid_pressure - self.p0) / (self.rho * self.g)



# ================= COMPUTE =================

    def compute_pose(self):
        odom = Odometry()
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