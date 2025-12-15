#!/usr/bin/env python3
# -- coding: utf-8 --


import rospy
import numpy as np
from sensor_msgs.msg import Imu, FluidPressure
class processamento:
    def __init__(self):

        # ===== Setup ROS =====

        rospy.init_node('processamento_sensor', anonymous=True)

        # Publisher
        self.imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        self.pressure_pub = rospy.Publisher('pressure_data', FluidPressure, queue_size=10)

        # Subscriber
        self.imu_sub = rospy.Subscriber('imu_raw', Imu, self.imu_callback)
        self.pressure_sub = rospy.Subscriber('pressure_raw', FluidPressure, self.pressure_callback)

        self.rate = rospy.Rate(10)  # Hz


        # ===== Parâmetros =====
        

        # Filtros de média móvel (exponencial)
        self.alpha_imu = 0.9
        self.alpha_pressure = 0.95

        # Offsets para calibração
        self.accel_offset = [0.0, 0.0, 9.81] 
        self.angular_velocity_offset = [0.0, 0.0, 0.0]
        self.orientation_offset = [0.0, 0.0, 0.0, 0.0]
        self.pressure_offset = 0.0

        # Variáveis internas
        self.filtered_imu = Imu()
        self.filtered_pressure = FluidPressure()


    # ================= CALLBACKS =================

    def imu_callback(self, msg):
        raw_imu = msg

        # ===== Aceleração =====
        accel = np.array([
            raw_imu.linear_acceleration.x,
            raw_imu.linear_acceleration.y,
            raw_imu.linear_acceleration.z
        ]) - self.accel_offset

        prev = np.array([
            self.filtered_imu.linear_acceleration.x,
            self.filtered_imu.linear_acceleration.y,
            self.filtered_imu.linear_acceleration.z
        ])

        accel_f = self.alpha_imu * prev + (1 - self.alpha_imu) * accel

        self.filtered_imu.linear_acceleration.x = accel_f[0]
        self.filtered_imu.linear_acceleration.y = accel_f[1]
        self.filtered_imu.linear_acceleration.z = accel_f[2]
        
        # ===== velocidade angular =====
        vel = np.array([
            raw_imu.angular_velocity.x,
            raw_imu.angular_velocity.y,
            raw_imu.angular_velocity.z
        ]) - self.angular_velocity_offset

        prev_vel = np.array([
            self.filtered_imu.angular_velocity.x,
            self.filtered_imu.angular_velocity.y,
            self.filtered_imu.angular_velocity.z
        ])

        vel_f = self.alpha_imu * prev_vel + (1 - self.alpha_imu) * vel

        self.filtered_imu.angular_velocity.x = vel_f[0]
        self.filtered_imu.angular_velocity.y = vel_f[1]
        self.filtered_imu.angular_velocity.z = vel_f[2]

        # ===== Orientação =====
        orientation = np.array([
            raw_imu.orientation.x,
            raw_imu.orientation.y,
            raw_imu.orientation.z,
            raw_imu.orientation.w
        ]) - self.orientation_offset

        prev_orient = np.array([
            self.filtered_imu.orientation.x,
            self.filtered_imu.orientation.y,
            self.filtered_imu.orientation.z,
            self.filtered_imu.orientation.w
        ])
        
        orient_f = self.alpha_imu * prev_orient + (1 - self.alpha_imu) * orientation
        
        self.filtered_imu.orientation.x = orient_f[0]
        self.filtered_imu.orientation.y = orient_f[1]
        self.filtered_imu.orientation.z = orient_f[2]
        self.filtered_imu.orientation.w = orient_f[3]

        # Publish filtered IMU

        self.imu_pub.publish(self.filtered_imu)
        

    def pressure_callback(self, msg):
        raw_pressure = msg.fluid_pressure - self.pressure_offset
        prev_pressure = self.filtered_pressure.fluid_pressure
        
        pressure_f = self.alpha_pressure * prev_pressure + (1 - self.alpha_pressure) * raw_pressure 
        self.filtered_pressure.fluid_pressure = pressure_f

        self.pressure_pub.publish(self.filtered_pressure)
    
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = processamento()
        node.run()
    except rospy.ROSInterruptException:
        pass