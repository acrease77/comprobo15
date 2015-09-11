#!/usr/bin/env python

"""Experiment with recieveing ROS_messages"""

import rospy
from geometry_msgs.msg import PointStamped

def process_point(msg):
	print msg

rospy.init_node('receive_messages')
rospy.Subscriber('/my_point', PointStamped, process_point)