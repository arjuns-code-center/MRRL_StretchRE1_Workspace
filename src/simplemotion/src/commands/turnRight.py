#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('turn_right')
pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    command = Twist()
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = -0.1

    pub.publish(command)
    rate.sleep()