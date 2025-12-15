#!/usr/bin/env python3
# -- coding: utf-8 --


import rospy
from std_msgs.msg import String

class Comunicacao:
    def __init__(self):
        rospy.init_node('comunication_node', anonymous=True)

        # Publisher
        self.pub = rospy.Publisher('topic_out', String, queue_size=10)

        # Subscriber
        self.sub = rospy.Subscriber('topic_in', String, self.callback)

        self.rate = rospy.Rate(10)  # Hz

    def callback(self, msg):
        rospy.loginfo(f"Recebido: {msg.data}")
        self.pub.publish(f"Eco: {msg.data}")

    def run(self):
        while not rospy.is_shutdown():
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Comunicacao()
        node.run()
    except rospy.ROSInterruptException:
        pass