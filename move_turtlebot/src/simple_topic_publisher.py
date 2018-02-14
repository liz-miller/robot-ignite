#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist

#initialize the node
rospy.init_node('move_turtlebot_node')

# create the publisher
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
move = Twist()
rate = rospy.Rate(2)

move.linear.x = 1.0
move.angular.z = 0.0
i = 0

while not rospy.is_shutdown() and i <= 10:
    rospy.loginfo("Robot starting to move")
    pub.publish(move) #publish the move
    rate.sleep()
    i += 1
    if i >= 10:
        rospy.loginfo("Robot stopping...")
        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)
        rate.sleep()