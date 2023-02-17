#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class PID(): 
    '''
    single variable PID controller
    takes parameters, final distance, maximum velocity and timestamp
    '''
    def __init__(self, Kp:float, Ki:float, Kd:float, xtot:float, vmax:float, dt:float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.xtot = xtot   
        self.vmax = vmax
        self.dt = dt
        self.err = 0
        self.integral = 0
        
 

    def integrate(self, err):
        self.integral = self.integral + (err)*(self.dt) #add step to integral and return
        print(self.integral)
        return self.integral

    def differentiate(self, err):
        derivative = (err - self.err)/self.dt #calculte differential
        print(derivative)
        return derivative
    
    def findVout(self, xin:float):
        '''
        return new velocity, given displacement 
        '''
        err = self.xtot - xin
        vout = self.Kp*err + self.Ki*self.integrate(err) + self.Kd*self.differentiate(err)
        self.err = err
        if (vout < self.vmax):
            return vout
        else:
            return self.vmax


vKp = 0.05
vKi = 0
vKd = 0
finalX = 5
finalY = 0
vdt = 0.0333
vmax = 0.2

omegaKp = 0
omegaKi = 0
omegaKd = 0
finalOmega = 0
omegadt = 0


# class main():

#     def _init_(self):
#         #define variables
#         self.x
#         self.y
#         self.theta
#         self.v
#         self.omega
        

#         #get intial position and 
#         self.getPose()
#         self.vPID = PID(vKp, vKi, vKd, finalX, vdt)
#         self.omegaPID = PID(omegaKp, omegaKi, omegaKd, finalOmega, omegadt)


#         rospy.init_node('PID', anonymous = True)
#         rospy.Subscriber('/odom', Odometry, self.getPose())
#         self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)


#     def getPose(self, msg):
#         self.x = msg.pose.pose.position.x 
#         self.y = msg.pose.pose.position.y
#         self.v = msg.twist.twist.linear.x
#         self.omega = msg.twist.angular.z
#         rot_q = msg.pose.pose.orientation
#         (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

#     def pubVel(self):
#         newV = self.vPID.findVout(self.v, self.d)
#         newOmega = self.omegaPID.findVout(self.omega, self.theta)




class main():
    def __init__(self):
        #instiate msg in init once or new one every time in function?
        self.velocity = Twist()
        self.vPID = PID(vKp, vKi, vKd, finalX, vmax, vdt)
        print(vdt)
        rospy.init_node('PID', anonymous = True)
        rospy.Subscriber('/odom', Odometry, self.getPose)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.spin()

    def getPose(self, msg):
        x = msg.pose.pose.position.x 
        y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.findNewCmd(x)

    def findNewCmd(self, x):
        #take v from function or class variable?
        newV = self.vPID.findVout(x)
        self.pubCmd(newV, 0)

    def pubCmd(self, v, theta):
        self.velocity.linear.x = v
        self.velocity.angular.z = theta

        self.pub.publish(self.velocity)
        print("publishing", v)



if __name__ == '__main__':
    main()

