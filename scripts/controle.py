import rospy
import math

import tf
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Controle:
    def __init__(self):

        # ===== Setup ROS =====

        # ROS Setup
        rospy.init_node('Control_node', anonymous=True)

        # Publishers
        self.motor_pub  = rospy.Publisher('\motor_cmd', Float32MultiArray, queue_size=10)

        # Subscribers
        self.sub_state = rospy.Subscriber('\state', Odometry, self.state_callback)
        self.sub_input = rospy.Subscriber('\cmd_vel', Twist, self.cmd_callback)

        self.rate = rospy.Rate(10)  # Hz


        # ===== Parâmetros =====

        # Constantes PID 
        self.kp_yaw = 1.0
        self.ki_yaw = 0.0
        self.kd_yaw = 0.0

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
        Implementa o controle PID para yaw e profundidade.
        Calcula os sinais de controle.
        """
        
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt == 0:
            return
        
        # Extrai estado atual
        current_yaw = self.get_yaw_from_odometry(self.state)
        current_depth = self.state.pose.pose.position.z

        # Extrai comandos desejados
        desired_yaw = self.cmd.angular.z
        desired_depth = self.cmd.linear.z

        # Calcula erros
        yaw_error = desired_yaw - current_yaw
        depth_error = desired_depth - current_depth

        # Integração dos erros
        self.yaw_error_i += yaw_error * dt
        self.depth_error_i += depth_error * dt

        # Derivada dos erros
        yaw_error_d = yaw_error / dt
        depth_error_d = depth_error / dt

        # Sinais de controle PID
        yaw_control = (self.kp_yaw * yaw_error +
                       self.ki_yaw * self.yaw_error_i +
                       self.kd_yaw * yaw_error_d)
        
        depth_control = (self.kp_depth * depth_error +
                         self.ki_depth * self.depth_error_i +
                         self.kd_depth * depth_error_d)
        
        return [yaw_control, depth_control]

    def distribute_motor_commands(self, yaw_control, depth_control):
        """
        Distribui os sinais de controle para os motores do ROV.
        """
        motor_cmd = Float32MultiArray()

        # exemplo basico com 4 motores (2 para profundidade e 2 para avançar e rotacionar)
        # Falta o sinal de avanço do ROV
        motor_cmd.data = [depth_control,
                          depth_control,
                          yaw_control,
                          -yaw_control]
        return motor_cmd
        
    def get_yaw_from_odometry(self, odom):
        """
        Extrai o ângulo yaw (em radianos) da mensagem de odometria.
        """
        orientation_q = odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw
    
    # ================= LOOP =================

    def run(self):
        while not rospy.is_shutdown():
            yaw_control,depth_control = self.compute_control()     
            motors = self.distribute_motor_commands(yaw_control, depth_control)      
            self.motor_pub.publish(motors) 
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Controle()
        node.run()
    except rospy.ROSInterruptException:
        pass
