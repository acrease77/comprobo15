#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class robot_control():
    '''
    control robot in wall following mode
    '''
    def __init__(self):
        '''
            initialize ros node and variables which are accessed by multiple functions
        '''
         rospy.init_node('estop', anonymous=True) #initialize node
        rospy.Subscriber('/bump', Bump, self.bump_callback) #setup callback for bump sensor
        rospy.Subscriber('/scan', LaserScan, self.laser_callback) #setup callback for laser scaner
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0) #x value = forward speed
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0) #z value = turn rate

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #initialize control of robot
        
        self.goal=90 #walol follow target angle
        self.state = 'find_wall' #initialize state of robot
        self.turn_start = 0.0 #value for where to start turning away from corners
        self.data=() #data from lidar

    def bump_callback(self, data):
        '''
            callback for bump sensor
        '''
        sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
        if self.state != 'hit_wall':
            if sensor > 0: #during follow wall, if bumper is hit, start backing up and swich mode to hit wall
                self.lin_vector.x=-0.25
                self.send_command()
                self.state = 'hit_wall'
        else:
            if sensor == 0: #once the robot is off the wall, change mode to corner hit (to turn away from the wall)
                self.stop()
                self.state='corner_hit'


    def laser_callback(self, data):
            '''
            set self.data to the list of distances from the laser scanner 
            when a scan is received
            '''
            self.data=data.ranges

    def find_wall(self):
        '''
            decide which side of the robot a wall is on
            only runs once, immediately before 'wall_align'
        '''
        dists = []
        for i in range(8):
            #get the average distance of objects in 45 degree wedges
            dists.append(self.read_angle(45*i+22))
        wall_dist = min(dists) #find wedge with the closest object
        angle = dists.index(wall_dist) #find which wedge that is
        if angle<4: #if the closest object is on the left, set target angle to 90
            self.goal=90
        else: #if the closest object is on the right, set target angle to 270
            self.goal=270
        self.state='follow_wall' #change state to 'follow wall'

    def wall_align(self):
        '''
            follow a wall:
            read the distance to the wall at (target) plus and minus 40 degrees
            use proportional control to make the two readings equal
            also, make the distance at both angles equal to 0.75 m
        '''
        ang_k=-1 #coeficient for spin relating to the angles
        if self.goal == 90:
            dist_k = 0.5 #coeficient relating to distance with wall on the 
        else:
            dist_k = -0.5 #codficient relating to distance with wall on the right
        c=0.03 #coeficient for speed
        dist_target = 0.75 #target distance from wall
        dist1=self.read_angle(self.goal+40) #read the distance at the two angles
        dist2=self.read_angle(self.goal-40)
        ang_err = dist1-dist2 #computers
        avg_dist = (dist1+dist2)/2 #average distances
        dist_err = avg_dist-dist_target
        spin = ang_err*ang_k+dist_err*dist_k #first term corrects for angle to wall, second corrects for distnace from wall
        if ang_err == 0: #avoid divide by zero
            forward = 1
        else:
            forward = abs(c/ang_err)+0.1 #control speed based on angle error (+0.1 to stay above dead zone)
        self.set_motion(forward=forward, spin=spin) #send command to robot

    def smart_turn(self):
        '''
        turns away from corner in the case of a corner hit
        '''        
        if self.goal == 90: #turns away from corner depending on which side the wall is on
            self.set_motion(spin=-0.9)
        else:
            self.set_motion(spin=0.9)
        time.sleep(1.5)     #turns for 1.5 seconds, about 90 degrees
        self.state='follow_wall' #follows wall after corner turn

    def read_angle(self, angle):
        '''
            read the distance at an angle by averaging the readings
            from the desired angle plus or minus 5 degrees disregarding
            non-readins (avoids laser scanner glitches)
        '''
        pad = 5
        readings = []
        for i in range(2*pad):
            index = (angle - pad + i)%360
            dist = self.data[index]
            if dist > 0 and dist < 1.5:
                readings.append(dist)
        if readings != []:
            #normally return the average reading
            avg = sum(readings)/len(readings)
            return avg
        else:
            #if there were no valid readings, return 10 (larger than any readings we saw)
            return 10

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

    def main_loop(self):
        '''
            main run loop, runs correct function based on current state
        '''         
        print self.state #print state
        if self.state == 'find_wall':
            if self.data != ():
                print 'wall align'
                self.find_wall()
        elif self.state == 'follow_wall':
            self.wall_align()
        elif self.state == 'corner_hit':
            self.smart_turn()

if __name__=='__main__':
    neato = Robot_Control() #initialize robot
    r=rospy.Rate(10) #setup ros loop
    while not rospy.is_shutdown():
        neato.main_loop() #run main loop continuously
        r.sleep()