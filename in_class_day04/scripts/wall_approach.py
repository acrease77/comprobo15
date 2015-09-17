#!/usr/bin/env python
"""This is a ROS node that approaches a wall using proportional control"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class WallApproach():
    def __init__(self):
        rospy.init_node('wall_approach')
        rospy.Subscriber('/scan',LaserScan,self.process_scan)
        self.actual_dist = -1
        self.target = rospy.get_param('~target_distance')

        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    def process_scan(self, msg):
        print msg.ranges[0]
        if msg.ranges[0] != 0:
            self.actual_dist = msg.ranges[0]

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            error = self.actual_dist - self.target
            self.lin_vector.x = error
            self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))
            r.sleep()

if __name__ == '__main__':
    node = WallApproach()
    node.run()