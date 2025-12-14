import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32  
from sensor_msgs.msg import Imu

class Pose:
    def __init__(self):

        # ===== Setup ROS =====

        rospy.init_node('Pose_node', anonymous=True)

        # Publisher
        self.state_pub = rospy.Publisher('\state', Odometry, queue_size=10)

        # Subscriber
        self.imu_sub = rospy.Subscriber('\imu_data', Imu, self.Imu_callback)
        self.depth_sub = rospy.Subscriber('\depth', Float32, self.depth_callback)

        self.rate = rospy.Rate(10)  # Hz


        # ===== Parâmetros =====

        self.imu = Imu()
        self.depth = Float32()

 # ================= CALLBACKS =================


    def Imu_callback(self, msg):
        self.imu = msg
    def depth_callback(self, msg):
        self.depth = msg



# ================= COMPUTE =================

    def compute_pose(self):
        odom = Odometry()
        # Preencher odom com os dados do imu e depth
        # Exemplo simplificado:
        odom.pose.pose.orientation = self.imu.orientation
        odom.pose.pose.position.z = self.depth.data
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