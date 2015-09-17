#!/usr/bin/env python

import rospy
import time
import math
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
        self.state = 'obstacle_avoid'
        self.turn_start = 0.0
        self.data = ()
        self.set_dist = .5

    def cartesian_convert(self,dist,angle):
        force_strength = 1/dist
        forward_vector = force_strength*math.cos(angle)
        side_vector = force_strength*math.sin(angle)
        vector = [forward_vector,side_vector]
        return vector

    def obstacle_arbiter(self,readings):
        sum_vectors = [0.0,0.0]
        for vector in readings:
            force = self.cartesian_convert(vector[1],vector[0])
            sum_vectors[0]+=force[0]/len(readings)
            sum_vectors[1]+=force[1]/len(readings)
        sum_vectors[0]+=.3
        return sum_vectors

    def obstacle_avoid(self):
        things = self.find_things()
        vector = self.obstacle_arbiter(things)
        k= -5
        c= 1.5
        forward = c*vector[0]
        spin = k*vector[1]
        self.set_motion(forward=forward,spin=spin)

    def find_wall(self):
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

    def find_things(self):
        while self.data == ():
            pass
        pad = 179
        readings = []
        for i in range(2*pad):
            index = (360-pad+i)%360
            dist = self.data[index]
            readings.append(dist)
        variable_name = list(enumerate(readings))
        body_reading = []
        for reading in variable_name:
            if reading[1] > .5 and reading[1] < 1.5:
                body_reading.append(reading)
        angle_sum=0.0
        dist_sum=0.0
        for point in body_reading:
            angle_sum+=point[0]
            dist_sum+=point[1]
        if self.state == 'follow_person':
            if len(body_reading) == 0:
                return (0.0, self.set_dist)
            avg_angle = angle_sum/len(body_reading)
            avg_dist = dist_sum/len(body_reading)
            return (avg_angle-45, avg_dist)
        if self.state == 'obstacle_avoid':
            return body_reading

    def follow_person(self):
        (angle, distance) = self.find_things()
        k= -1.27
        c= 1.5
        dist_diff = distance - self.set_dist
        angle_diff = -angle
        forward = c*dist_diff
        spin = k*angle_diff/45
        print dist_diff
        #print forward
        self.set_motion(forward=forward,spin=spin)


    def wall_align(self):
        k= 1.27
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
        #print self.lin_vector.x

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
            if self.state == 'follow_person':
                self.follow_person()
            if self.state == 'obstacle_avoid':
                self.obstacle_avoid()

if __name__=='__main__':
    neato = Robot_Control()
    r=rospy.Rate(10)
    while not rospy.is_shutdown():
        neato.main_loop()
        r.sleep()