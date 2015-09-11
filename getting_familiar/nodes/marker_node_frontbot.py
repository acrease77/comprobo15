#!/usr/bin/env python

"""publishes message to put marker 2 meters in front of the robot in rvis odom"""

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

rospy.init_node('marker_node_front_bot')

def update_message():		#adds marker
	point_msg = Point(x=2.0, y=0.0, z=0.0)	#adds marker 2 meters in front of robot
	pose_msg = Pose(position=point_msg)
	scale_msg = Vector3(x=.5, y=0.5, z=0.5)		#marker size
	color_msg = ColorRGBA(r=1.0, g = 1.0, b=1.0, a=.5)		#marker color

	#adds marker in front of robot

	header_msg = Header(stamp=rospy.Time.now(),
					    frame_id="base_link")

	msg = Marker(header=header_msg, pose=pose_msg,scale=scale_msg,color=color_msg)
	msg.type=2
	return msg

pub = rospy.Publisher("/my_point", Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	msg = update_message()		#updates marker continuously to follow the robot
	pub.publish(msg)
	r.sleep()