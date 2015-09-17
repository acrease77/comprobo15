#!/usr/bin/env python
"""This is a ROS node that approaches a wall using proportional control"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

class OdomPropControl():
    def __init__(self):
        rospy.init_node('odom_prop_control')
        rospy.Subscriber('/odom',Odometry,self.process_odom)

        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.x0 = None
        self.target_x = None
        self.current_x = None
        self.k = - 0.1

    def process_odom():
        if self.x0 == None:
            self.x0 = msg.pose.pose.position.x
            self.target_x = self.x0 + 1.0
        self.current_x = msg.post.post.position.x

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print self.current_x, self.target_x
            if self.current_x != None and self.target_x != None:
                error = self.current_x - self.target_x
                msg = Twist(linear=Vector3(x=error*self.k))
                self.pub.publish(msg)
            r.sleep()

if __name__ == '__main__':
    node = WallApproach()
    node.run()