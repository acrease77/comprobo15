#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String 
from neato_node.msg import Bump

class robot_control():
	def __init__(self):
		rospy.init_node('estop',anonymous=True)
		rospy.Subscriber('bump',Bump,self.callback)

		self.value_lin = Vector3(x=0.0,y=0.0,z=0.0)
		self.value_ang = Vector3(x=0.0,y=0.0,z=0.0)

		self.pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)

	def forward(self):
		print 'forward'
		self.value_lin.x = 0.5

	def callback(self,data):
		sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
		if sensor > 0:
			self.value_lin.x=-0.5
			self.publish()
		if sensor == 0 and self.value_lin.x<0:
			self.stop()
			self.publish()


	def publish(self):
		self.pub.publish(Twist(linear=self.value_lin,angular=self.value_ang))

	def stop(self):
		print 'STHAAAPPPPP'
		self.value_lin.x=0
		self.value_ang.x=0

if __name__=='__main__':
	neato = robot_control()
	neato.forward()
	time.sleep(1)
	neato.publish()
	rospy.spin()
