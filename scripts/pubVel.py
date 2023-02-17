#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class controller:
    def __init__(self):
        rospy.init_node("controller")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.velocity = Twist()
        self.velocity.linear.x = 0.1

        while not rospy.is_shutdown():
            self.pub.publish(self.velocity)

        # self.pub.publish(self.velocity)
        # rospy.spin()

if __name__ == '__main__':
    controller()