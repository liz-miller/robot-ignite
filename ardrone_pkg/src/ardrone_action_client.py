#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

nImage = 1

# State options
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

# initializes the action client node
rospy.init_node('ardrone_action_client')

# create the connection to the action server
client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move = Twist()

#create a takeoff message
takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
takeoff_msg = Empty()

#create a landing message
land = rospy.Publisher('/drone/land', Empty, queue_size=1)
land_msg = Empty()

# wait until action server is up and running
rospy.loginfo('Waiting for action Server '+'/ardrone_action_server')
client.wait_for_server()
rospy.loginfo('Action Server Found...'+'/ardrone_action_server')

# creates a goal to send to the action server
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)

# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time 
# status = client.get_state()
# check the client API link below for more info

#client.wait_for_result()

# SimpleAction variable state check
state = client.get_state()
rate = rospy.Rate(1)

#TAKE OFF
i = 0
while not i == 3:
    rospy.loginfo('Initiate TAKEOFF')
    takeoff.publish(takeoff_msg)
    time.sleep(1)
    i += 1

while state < DONE:
    move.linear.x = 1.0
    move.angular.z = 1.0
    move_pub.publish(move)
    rate.sleep()
    state = client.get_state()
    rospy.loginfo('Moving Around...')
    rospy.loginfo('state: '+str(state))

rospy.loginfo('[Result] State: '+str(state))
if state == ERROR or state == DONE:
    rospy.logerr('Something went wrong in the Server Side')

if state == WARN:
    rospy.logwarn('There is a warning in the Server Side')

# LANDING
i = 0
while not i == 3:
    move.linear.x = 0.0
    move.angular.z = 0.0
    move_pub.publish(move)
    rospy.loginfo('Stopping...')
    land.publish(land_msg)
    rospy.loginfo('Initiating LANDING')
    time.sleep(1)
    i += 1

