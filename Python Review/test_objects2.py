#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

class CrazyBot:
	"""represents a crazy robot with specified forward and reverse velocities"""
	def __init__(self,velocity):
		self.velocity = velocity
		rospy.init_node('test_objects2')
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		rospy.Subscriber("/bump", Bump, self.process_bump) #define a method in class called process_bump that subscribes to bump
		self.should_stop = False

	def process_bump(self,msg):
		if msg.leftFron == 1 or msg.rightFront == 1:
			twist_message = Twist()
			self.pub.publish(twist_message)
			self.should_stop = True
		else:
			self.should_stop = False

	def run(self):
		r = rospy.Rate(1)
		i=0
		l_1 = Vector3(0.1,0.0,0.0)
		a_1 = Vector3(0.0,0.0,0.0)

		twist_message_1=Twist(l_1,a_1)

		while not rospy.is_shutdown():
			if i % 2 = 0:
				l_1.x = self.velocity		#this is referencing inputs, so the 
			else:
				l_1.x = self.velocity
			if not self.should_stop:
				self.pub.publish(twist_message_1)
			r.sleep()
			i += 1

node = CrazyBot(0.2)
node.run()