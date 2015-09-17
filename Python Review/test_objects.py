#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

rospy.init_node('test_objects')

l_1 = Vector3(0.1,0.0,0.0)
a_1 = Vector3(0.0,0.0,0.0)

twist_message_1=Twist(l_1,a_1)
print twist_message_1.linear

L-2 = Vector3(y=2.0)
print l_2

r = rospy.Rate(1)
i=0
while not rospy.is_shutdown():
	if i % 2 = 0:
		l_1.x = 0.3		#this is referencing inputs, so the 
	else:
		l_1.x = 0.0
	print "hello!"
	r.sleep()
	i += 1