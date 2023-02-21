#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PID():
    '''
    single variable PID controller
    initially takes parameters, maximum velocity and timestamp

    '''
    def __init__(self, Kp:float, Ki:float, Kd:float, vmax:float, dt:float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.vmax = vmax
        self.dt = dt
        self.err = 0
        self.integral = 0

    def reset(self):
        self.err = 0
        self.integral = 0

    def integrate(self, err):
        self.integral = self.integral + (err)*(self.dt) #add step to integral and return
        # print(self.integral)
        return self.integral

    def differentiate(self, err):
        derivative = (err - self.err)/self.dt #calculte differential
        # print(derivative)
        return derivative

    def findVout(self, err:float):
        '''
        return new velocity, given displacement
        '''
        vout = self.Kp*err + self.Ki*self.integrate(err) + self.Kd*self.differentiate(err)
        self.err = err
        if (vout < self.vmax):
            return vout
        else:
            return self.vmax

class main():
    def __init__(self):
        self.points = [(0, 5), (6, 2)]
        self.numOfNodes = len(self.points)
        self.x = 0
        self.y = 0
        self.theta = 0.0300867211620511
        self.whereTo = 0
        self.linearPIDflag = False
        self.velocity = Twist()

        vKp = 0.1
        vKi = 0.00
        vKd = 0.00
        vdt = 0.0333
        vmax = 0.2
        omegaKp = 0.1
        omegaKi = 0.00
        omegaKd = 0.00
        omegadt = 0.0333
        omegaMax = 2.8
        self.vPID = PID(vKp, vKi, vKd, vmax, vdt)
        self.omegaPID = PID(omegaKp, omegaKi, omegaKd, omegaMax, omegadt)

        rospy.init_node('PID', anonymous = True)
        rospy.Subscriber('/odom', Odometry, self.callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.spin()

    def callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        print(self.theta)
        x, y = self.points[self.whereTo]
        self.goTo(x, y)

    def goTo(self, x, y):
        theta = math.atan2((y- self.y), (x -self.x))
        if not self.reached(x, y):
            v = 0
            omega = self.omegaPID.findVout(theta - self.theta)
            print("goal", theta)
            print("current", self.theta)

            print(theta - self.theta)
            if not self.linearPIDflag:
                if self.headingRight(x, y):
                    self.linearPIDflag = True
            else:
                v = self.vPID.findVout(self.dist(x, y, self.x, self.y))
            self.pubCmd(v, omega)
        elif self.whereTo <= self.numOfNodes:
            self.vPID.reset()
            self.omegaPID.reset()
            self.whereTo += 1
            self.linearPIDflag = False
        else:
            self.pubCmd(0, 0)

    def pubCmd(self, v, omega):
       self.velocity.linear.x = v
       self.velocity.angular.z = omega
       self.pub.publish(self.velocity)
       print("publishing", v, omega)
       print(self.whereTo)
       print(self.linearPIDflag)

    def dist(self, x1, y1, x2, y2):
        dist = ((x2 - x1)**2+(y2-y1)**2)**0.5
        return dist

    def headingRight(self, x, y):
        theta = math.atan2(y, x)
        if abs(theta-self.theta) < 0.1:
            return True
        return False

    def reached(self, x, y):
        if self.dist(x, y, self.x, self.y) < 0.2:
            return True
        return False

if __name__ == '__main__':
    main()