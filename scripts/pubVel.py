#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



class controller:
    def __init__(self):
        rospy.init_node("controller")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.velocity = Twist()
        # self.velocity.linear.x = 0.1
        self.velocity.angular.z = 1
        




           
        rospy.Subscriber('/odom', Odometry, self.callback)
        # self.pub.publish(self.velocity)
        rospy.spin()
            
    def callback(self, msg):
        self.pub.publish(self.velocity)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        print(theta)


        # self.pub.publish(self.velocity)
        # rospy.spin()

if __name__ == '__main__':
    controller()