#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
'''from sensor_msgs.msg import Float32'''

import sys, select, termios, tty
range_center = Float32()
range_left = Float32()
range_right = Float32()
range_left_last = Float32()
range_right_last = Float32()

normal_vel =0.2
normal_angel_turn =1.57284
last_ob_vel =0
last_ob_angel =0.21845
half_angel =1.57284

def callback(data):

    rospy.loginfo("ranges %f", data.ranges[359])
    range_center.data= data.ranges[359] 
    range_left.data= data.ranges[180]
    range_right.data= data.ranges[540]
    range_left_last.data= data.ranges[20]
    range_right_last.data= data.ranges[700]

def move():
    '''turtlebot_teleopr'''
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    sub = rospy.Subscriber("scan", LaserScan, callback)
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    twist = Twist()
    while not rospy.is_shutdown():
	
		if (range_center.data>1):
			twist.linear.x =normal_vel; twist.linear.y = 0; twist.linear.z = 0
		    	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		    	pub.publish(twist)
			if (range_left_last.data<0.2):
				twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = last_ob_angel
				pub.publish(twist)

			elif (range_right_last.data<0.2):
				twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -last_ob_angel
				pub.publish(twist)
		    	print(range_center.data)

		else :
			if (range_left_last.data>1) or (range_left.data>1):
				twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -normal_angel_turn
				pub.publish(twist)


			elif (range_right_last.data>1) or (range_right.data>1) :
				twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = normal_angel_turn
				pub.publish(twist)

		    	elif (range_left.data>range_right.data):
			    	twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
			    	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -normal_angel_turn
			    	pub.publish(twist)
			    
		    	else :
			    	twist.linear.x =last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
			   	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = normal_angel_turn
			    	pub.publish(twist)

			   
                   
 		rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass


