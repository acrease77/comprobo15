#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String 
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class Robot_Control():
    '''
    Controls robot in obstacle avoidance mode
    '''
    def __init__(self):
        rospy.init_node('estop', anonymous=True)  #initialize node
        #rospy.Subscriber('/bump', Bump, self.bump_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback) #setup callback for laser scaner
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0) #x value = forward speed
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0) #z value = turn rate

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #initialize control of robot
        

        self.side=90 #wall follow target angle
        self.state = 'obstacle_avoid' #initialize state to obstacle avoid
        self.data = () #initialize variable to store laser scan data
        self.set_dist = .5 #set distacne from person


    def obstacle_avoid(self):
        '''
            avoid obstacles using proportional control
            groups objects into categories of front left, front right, back left, and back right
            reacts at a rate proportional to the distance to the obstacle
        '''
        things = self.find_things() #get list of angles and distances between 0.25 and 1.25 meters
        c= 0.75 #coeficient for forward speed
        k= 0.4 #coeficient for turn rate

        spin = 0
        dist_sum = 0
        for angle, distance in things:
            if angle-179 < -60:
                #pulls the robot slightly towards objects behind and to the left
                spin -= 0.2/distance #respond more to closer objects (for all cases)
                dist_sum += 2 #treat as a far away reading for speed calculation
            elif angle-179 < 0:
                #pushes robot away from objects in front and to the left
                spin += 5/distance
                dist_sum += distance-0.5
            elif angle-179 < 60:
                #pushes robot away from objects in front and to the right
                spin -= 5/distance
                dist_sum += distance-0.5
            elif angle-170 < 180:
                #pulls robot slightly towards objects behind and to the right
                spin += 0.2/distance
                dist_sum += 2
        if len(things) == 0: #avoid dividing by zero
            forward = 1
            spin = 0
        else:
            forward = c*dist_sum/len(things) #react to all objects the same way, regardless of number of readings
            spin = k*spin/len(things)
        self.set_motion(forward=forward,spin=spin) #send command to robot

    def find_things(self):
        '''Sweeps the area in .2-1.5 meters of the Neato with the lidar, and calculates the center of mass of the points it sees during find person
         -returns a tuple of the angle of the COM relative to the front of the robot and the distance of the COMfrom the robot for find person
         -returns a list of things it saw in its range during obstacle avoid'''
        while self.data == ():
            pass
        if self.state == 'obstacle_avoid':
            pad = 179 #the angle the robot sweeps to on either side, starting from the front
            max_dist = 1.25 #max of desired distacne range
        elif self.state == 'follow_person':
            pad = 35 #the angle the robot sweeps to on either side, starting from the front
            max_dist = 1.5
        readings = [] #array of lidar readings 
        for i in range(2*pad): #sweeps through the angles defined by pad
            index = (360-pad+i)%360     #calculates actual angle with respect to the robot's coordinate system
            dist = self.data[index]     #reads values from those angles
            readings.append(dist)       #adds to readings array
        variable_name = list(enumerate(readings))   #turns readings into a list of tuples containing their index and data value
        body_reading = []   #reading of points that define what the robot should follow
        for reading in variable_name:   #sweeps through readings from enumerated list
            if reading[1] > .25 and reading[1] < max_dist:    #adds all readings inside a specified box to the body_reading list
                body_reading.append(reading)
        angle_sum=0.0   #sum of all angles for COM calculation
        dist_sum=0.0    #sum of all distances for COM calculation
        for point in body_reading:  
            angle_sum+=point[0]     #sums up angles (still enumerations, but because of their nature are just offset angles)
            dist_sum+=point[1]  #sums up all distances
        if self.state == 'follow_person':
            if len(body_reading) == 0: #for first instance of no reading, intitialize start_time
                if self.start_time == 0.0:
                    self.start_time = time.time()
                else:
                    if time.time() - self.start_time > 3: #after 3 seconds of no valid readings, change state to obstacle avoid
                        self.state = 'obstacle_avoid'
                return (0.0, self.set_dist) #avoid divide by zero
            else: #if there is a valid reading, reset start_time
                self.start_time = 0.0
            avg_angle = angle_sum/len(body_reading) #get average angle
            avg_dist = dist_sum/len(body_reading) #get average distance
            return (avg_angle-35, avg_dist) #returns tuple of COM values and accounts for angle offset from enumerate
        if self.state == 'obstacle_avoid':
            return body_reading #for obstacle avoid, just return list of values in the set range

    def set_motion(self,forward=0,spin=0):
        '''
            send desired forward and spin commands to robot
        '''
        self.forward(forward)
        self.rotate(spin)
        self.send_command()


    def forward(self,rate):
        '''
            set the robots forward speed
            limited to max_speed
        '''
        max_speed = 0.5
        if abs(rate) < max_speed:
             self.lin_vector.x = rate
        else:
            self.lin_vector.x=rate/abs(rate)*max_speed

    def stop(self):
        ''' set values to stop robot '''
        print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0
        self.send_command()

    def rotate(self,rate):
        ''' 
        set rotation speed for the robot
        ensuring that no values over 1 are sent
        '''
        if abs(rate) < 1:
            self.ang_vector.z=rate
        else:
            self.ang_vector.z=rate/abs(rate)
              

     def send_command(self):
        '''publish command to robot'''
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

    def laser_callback(self, data):
            '''
            set self.data to the list of distances from the laser scanner 
            when a scan is received
            '''
            self.data=data.ranges

    def main_loop(self):
            '''
                main run loop, runs correct function based on current state
            ''' 
            #print self.state
            if self.state == 'obstacle_avoid':
                self.obstacle_avoid()

if __name__=='__main__':
    neato = Robot_Control() #initialize robot
    r=rospy.Rate(10) #setup ros loop
    while not rospy.is_shutdown():
        neato.main_loop() #run main loop continuously
        r.sleep()