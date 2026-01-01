#!/usr/bin/env python3
# -- coding: utf-8 --


import rospy
from dynamic_reconfigure.server import Server
from rov_control.cfg import PIDConfig

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
        pid = rospy.get_param("~pid")
        self.kp = pid['kp']
        self.ki = pid['ki']
        self.kd = pid['kd']
        
        self.server = Server(PIDConfig, self.dyn_reconf_callback)

        # Limites de sinal
        self.max_signal = rospy.get_param('~max_signal')
        self.min_signal = rospy.get_param('~min_signal')
        
        
        # variaveis internas
        self.cmd = Twist()
        self.state = Odometry()

        self.last_time = rospy.Time.now()

        # Erros PID
        self.depth_error_acc = 0.0
        self.last_depth_error = 0.0

        self.depth_control = 0.0


 # ================= CALLBACKS =================
    def dyn_reconf_callback(self, config, level):
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd

        rospy.loginfo(
            f"PID atualizado: kp={self.kp}, ki={self.ki}, kd={self.kd}"
        )
        return config

    def state_callback(self, msg):
        self.state = msg
    
    def cmd_callback(self, msg):
        self.cmd = msg

    # ================= UTIL =================

    def clip_signal(self, signal):
        """
        Limita o sinal de controle aos valores máximos e mínimos permitidos.
        """
        if signal > self.max_signal:
            return self.max_signal
        elif signal < self.min_signal:
            return self.min_signal
        else:
            return signal
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

        self.depth_error_acc += depth_error * dt
        depth_error_d = (depth_error - self.last_depth_error) / dt if dt > 0 else 0

        # Sinais de controle PID
        self.depth_control = self.kp * depth_error + self.ki * self.depth_error_acc + self.kd * depth_error_d
        self.depth_control = self.clip_signal(self.depth_control)


    def distribute_motor_commands(self):
        """
        Distribui os sinais de controle para os motores do ROV.
        """
        forward = self.cmd.linear.x
        rotate = self.cmd.angular.z
        depth_control = self.depth_control
        
        if rotate != 0 and forward != 0:
            left = (3*forward + rotate)/3
            right = (3*forward - rotate)/3
        else:
            left = forward + rotate
            right = forward - rotate
    
        motor_cmd = Float32MultiArray()
        motor_cmd.data = [left, right, depth_control] 
        # Falta o sinal de avanço do ROV
        self.motor_pub.publish(motor_cmd) 
        

    # ================= LOOP =================

    def run(self):
        while not rospy.is_shutdown():
            self.compute_control()
            self.distribute_motor_commands()      
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Controle()
        node.run()
    except rospy.ROSInterruptException:
        pass
