#!/usr/bin/env python3
# -- coding: utf-8 --


import rospy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Controle:
    def __init__(self):

        # ===== Setup ROS =====

        # ROS Setup
        rospy.init_node('Control_node', anonymous=True)

        # Publishers
        self.motor_pub  = rospy.Publisher('motor_cmd', Float32MultiArray, queue_size=10)

        # Subscribers
        self.sub_state = rospy.Subscriber('state', Odometry, self.state_callback)
        self.sub_input = rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)

        self.rate = rospy.Rate(10)  # Hz


        # ===== Parâmetros =====

        # Constantes PID 
        self.kp_depth = 1.0
        self.ki_depth = 0.0
        self.kd_depth = 0.0

        # variaveis internas

        self.cmd = Twist()
        self.state = Odometry()

        self.last_time = rospy.Time.now()

        # Erros PID
        self.yaw_error_i = 0.0
        self.depth_error_i = 0.0


 # ================= CALLBACKS =================


    def state_callback(self, msg):
        self.pose = msg
    
    def cmd_callback(self, msg):
        self.cmd = msg

    # ================= CONTROLE =================


    def compute_control(self):
        """
        Implementa o controle PID para profundidade.
        Calcula os sinais de controle.
        """
        
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt == 0:
            return
        

        current_depth = self.state.pose.pose.position.z
        desired_depth = self.cmd.linear.z

        # Calcula erros
        depth_error = desired_depth - current_depth
        self.depth_error_i += depth_error * dt
        depth_error_d = depth_error / dt

        # Sinais de controle PID
        
        depth_control = self.kp_depth * depth_error + self.ki_depth * self.depth_error_i + self.kd_depth * depth_error_d
        
        return depth_control

    def distribute_motor_commands(self, forward, rotate, depth_control):
        """
        Distribui os sinais de controle para os motores do ROV.
        """
        
        if rotate != 0 and forward != 0:
            left = (3*forward + rotate)/3
            right = (3*forward - rotate)/3
        else:
            left = forward + rotate
            right = forward - rotate
    
        motor_cmd = Float32MultiArray()
        motor_cmd.data = [left, right, depth_control] 
        # Falta o sinal de avanço do ROV
        
        return motor_cmd

    # ================= LOOP =================

    def run(self):
        while not rospy.is_shutdown():
            forward = self.cmd.linear.x
            rotate = self.cmd.angular.z
            depth_control = self.compute_control()
            motors = self.distribute_motor_commands(forward, rotate, depth_control)      
            self.motor_pub.publish(motors) 
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Controle()
        node.run()
    except rospy.ROSInterruptException:
        pass
