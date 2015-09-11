#!/usr/bin/env python

"""publishes message to put marker at (1,2) in rvis odom"""

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

rospy.init_node('marker_node')

point_msg = Point(x=1.0, y=2.0, z=0.0)
pose_msg = Pose(position=point_msg)
scale_msg = Vector3(x=1.0, y=1.0, z=1.0)
color_msg = ColorRGBA(r=1.0, g = 1.0, b=1.0, a=.5)

header_msg = Header(stamp=rospy.Time.now(),
				    frame_id="odom")

msg = Marker(header=header_msg, pose=pose_msg,scale=scale_msg,color=color_msg)
Marker.type=2
pub = rospy.Publisher("/my_point", Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	pub.publish(msg)
	r.sleep()

print 'hello'