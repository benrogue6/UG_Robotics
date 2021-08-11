#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#def callback(self, msg):
def callback(msg):
#def callback(msg):
 print msg.ranges[90]
 move.linear.x = 0.5
 if msg.ranges[90]<70:
  move.linear.x = 0
 pub.publish(move)

rospy.init_node ('check')
sub = rospy.Subscriber("/scan", LaserScan, callback) 
pub = rospy.Publisher("/cmd_vel", Twist)
move = Twist()

rospy.spin()

#self.twist.linear.x = 0.2
 #       self.twist.angular.z = -float(err) / 100
  #      self.cmd_vel_pub.publish(self.twist)
