#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String 
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class Robot_Control():
    def __init__(self):
        rospy.init_node('estop', anonymous=True)
        #rospy.Subscriber('/bump', Bump, self.bump_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.side=90
        self.state = 'find_wall'
        self.turn_start = 0.0
        self.data = ()


    def find_wall(self):
    '''sweeps through angles and finds the closest distance to wall, setting it as the distance to stay from the wall and the "goal angle" to stay perpendicular to the wall'''
        dists = []
        for i in range(8):
            dists.append(self.read_angle(45*i+22))
        smallest = min(dists)
        angle = dists.index(smallest)
        if angle < 4:
            self.goal = 90
        else:
            self.goal = 270
        self.state = 'follow_wall'


    def wall_follow(self):
    '''gets the distance away from the wall on either side of the goal angle, and follows the wall by attempting to keep it constant'''
        k= -1.27
        c=.03
        dist1 = self.read_angle(self.goal+40)
        dist2 = self.read_angle(self.goal-40)
        difference = dist1-dist2
        if difference == 0:
            forward = 1
        else:
            forward = abs(c/difference)
        #print difference
        #print forward
        self.set_motion(forward=forward,spin=k*difference)

    def read_angle(self, angle):
        pad = 5
        readings = []
        for i in range(2*pad):
            index = (angle - pad + i)%360
            dist = self.data[index]
            if dist > 0:
                readings.append(dist)
        if readings != []:
            avg = sum(readings)/len(readings)
            return avg
        else:
            return 10

    def set_motion(self,forward=0,spin=0):
        self.forward(forward)
        self.rotate(spin)
        self.send_command()


    def forward(self,rate):
        max_speed = 0.5
        if abs(rate) < max_speed:
             self.lin_vector.x = rate
        else:
            self.lin_vector.x=rate/abs(rate)*max_speed
        print self.lin_vector.x

    def stop(self):
        print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0
        self.send_command()

    def rotate(self,rate):
        max_speed = 0.5
        if abs(rate) < 1:
            self.ang_vector.z=rate
        else:
            self.ang_vector.z=rate/abs(rate) *max_speed
              

    def send_command(self):
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

    def laser_callback(self, data):
            self.data=data.ranges

    def main_loop(self):
            print self.state
            if self.state == 'find_wall':
                if self.data != ():
                    self.find_wall()
            if self.state == 'follow_wall':
                    self.wall_align()

if __name__=='__main__':
    neato = Robot_Control()
    r=rospy.Rate(10)
    while not rospy.is_shutdown():
        neato.main_loop()
        r.sleep()