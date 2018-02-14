#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
move = Twist()

def move_speed(x_speed, z_speed):
    move.linear.x = x_speed
    move.angular.z = z_speed

def callback(msg):
    if msg.ranges[360] < 1: #object within 1m
        rospy.loginfo("FRONT: Object Detected!")
        rospy.loginfo("Stopping ...")
        move_speed(0.0,0.0) #stop
        rospy.loginfo("Turning ...")
        move_speed(0.0, 0.25) #turn
    
    if msg.ranges[360] >= 1:
        rospy.loginfo("FRONT: No objects detected.")
        rospy.loginfo("Moving forward ...")
        move_speed(0.25,0.0) #moving
    
    if msg.ranges[0] < 1:
        rospy.loginfo("LEFT: Objects detected!")
        rospy.loginfo("Turning Right ...")
        move_speed(0.0,0.25) #right turn
    
    if msg.ranges[719] < 1:
        rospy.loginfo("RIGHT: Objects detected!")
        rospy.loginfo("Turning Left ...")
        move_speed(0.0,0.25) #right turn  
    
    pub.publish(move)
    
rospy.init_node('move_detect')
msg = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
rospy.spin()
    



