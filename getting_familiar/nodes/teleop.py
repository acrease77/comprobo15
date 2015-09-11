#!/usr/bin/env python

'''Drives the robot in teleop mode using w,s,a, and d commands'''

msg = """
Reading keyboard commands to drive the robot
---------------------------
w = Drive forward
D = Drive Backward
S = Turn left
A = Turn right
Anything Else = Stop

CTRL-C to quit
"""

import tty
import select
import sys
import termios
import time
import rospy
from geometry_msgs.msg import Twist, Vector3

rospy.init_node('marker_node')

#commands to drive the robot forward

forward_lin=Vector3(x=0.5,y=0.0,z=0.0)
forward_ang=Vector3(x=0.0,y=0.0,z=0.0)
forward_msg=Twist(linear=forward_lin, angular=forward_ang)

#command to drive the robot backward

backward_lin=Vector3(x=-0.5,y=0.0,z=0.0)
backward_ang=Vector3(x=0.0,y=0.0,z=0.0)
backward_msg=Twist(linear=backward_lin, angular=backward_ang)

#command to turn robot left

turn_left_lin=Vector3(x=0.0,y=0.0,z=0.0)
turn_left_ang=Vector3(x=0.0,y=0.0,z=1.0)
turn_left_msg=Twist(linear=turn_left_lin, angular=turn_left_ang)

#command to turn robot right

turn_right_lin=Vector3(x=0.0,y=0.0,z=0.0)
turn_right_ang=Vector3(x=0.0,y=0.0,z=-1.0)
turn_right_msg=Twist(linear=turn_right_lin, angular=turn_right_ang)

#command to make robot stop

stop_lin=Vector3(x=0.0,y=0.0,z=0.0)
stop_ang=Vector3(x=0.0,y=0.0,z=0.0)
stop_msg=Twist(linear=stop_lin, angular=stop_ang)

pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)

#get keypress from keyboard

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

#drive robot forward

def forward():
	print 'forward'
	return forward_msg

#drive robot back

def backward():
	print 'backward'
	return backward_msg

#turn robot left

def turn_left():
	print 'turn left'
	return turn_left_msg

#turn robot right

def turn_right():
	print 'turn right'
	return turn_right_msg

#stop robot

def stop():
	print 'STHAAAPPPPP'
	return stop_msg

#links keys to drive commands

def teleop():
	key = getKey()
	if key == 'w':
		pub.publish(forward())
	elif key == 'a':
		pub.publish(turn_left())
	elif key == 's':
		pub.publish(backward())
	elif key == 'd':
		pub.publish(turn_right())
	else:
		pub.publish(stop())
	time.sleep(0.01)
	return key

settings = termios.tcgetattr(sys.stdin)
key = None

print msg
r=rospy.Rate(10)
while key != '\x03':
    key = teleop()
    r.sleep()
