import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Controle:
    def __init__(self):

        # ROS Setup
        rospy.init_node('Controle', anonymous=True)

        # Publishers
        self.pub_control = rospy.Publisher('\motor_cmd', Float32MultiArray, queue_size=10)

        # Subscribers
        self.sub_state = rospy.Subscriber('\state', Odometry, self.callback_state)
        self.sub_input = rospy.Subscriber('\cmd_vel', Twist, self.callback_input)

        self.rate = rospy.Rate(10)  # Hz

        # Node variables

        self.pose = None
        self.input_cmd = None





    def callback_state(self, msg):
        pass
    
    def callback_input(self, msg):
        pass

    def run(self):
        while not rospy.is_shutdown():
            
            
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Controle()
        node.run()
    except rospy.ROSInterruptException:
        pass