#! /usr/bin/env python

import rospy
from move_turtlebot.msg import Age

rospy.init_node("age_pub")
pub = rospy.Publisher('age_info', Age, queue_size=3)
rate = rospy.Rate(12)

curr_age = Age()
curr_age.years = 1
curr_age.months = 4
curr_age.days = 23

while not rospy.is_shutdown():
    pub.publish(curr_age)
    rate.sleep()
