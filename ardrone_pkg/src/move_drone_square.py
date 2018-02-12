#! /usr/bin/env python
import rospy
import time
import actionlib
from actionlib.msg import TestFeedback, TestResult, TestAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class MoveSquareClass(object):
    
  # create messages that are used to publish feedback/result
  _feedback = TestFeedback()
  _result = TestResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("move_drone_square", TestAction, self.goal_callback, False)
    self._as.start()
    self.ctrl_c = False
    self.rate = rospy.Rate(10)
    
  def publish_once(self, cmd):
    while not self.ctrl_c:
        connections = self._move_pub.get_num_connections()
        if connections > 0:
            self._move_pub.publish(cmd)
            break
        else:
            self.rate.sleep()
    
    
  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    # this is the function that computes the Fibonacci sequence
    # and returns the sequence to the node that called the action server
    
    # helper variables
    r = rospy.Rate(1)
    success = True
    
     # create a move publisher & move message
    self._move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self._move = Twist()

    #create a takeoff message
    self._takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    self._takeoff_msg = Empty()

    #create a landing message
    self._land = rospy.Publisher('/drone/land', Empty, queue_size=1)
    self._land_msg = Empty()
    
    #make drone takeoff
    i = 0
    while not i == 3 and not self.ctrl_c:
        rospy.loginfo('Initiate TAKEOFF')
        self._takeoff.publish(self._takeoff_msg)
        time.sleep(1)
        i += 1
        
    sideSec = goal.goal
    turnSec = 1.8
    
    i = 0
    for i in xrange(0, 4):
       # publish the feedback - side # we're on
      self._feedback.feedback = i+1
      self._as.publish_feedback(self._feedback)
      
      # check that preempt (cancelation) has not been requested by the action client
      if self._as.is_preempt_requested():
        rospy.loginfo('Cancelling move request')
        # the following line, sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
        # we end the calculation of the Fibonacci sequence
        break
      
      # move forward
      self._move.linear.x = 1.0
      self._move.angular.z = 0.0
      self.publish_once(self._move)
      time.sleep(sideSec)
      
      # turn
      self._move.linear.x = 0.0
      self._move.angular.z = 1.0
      self.publish_once(self._move)
      time.sleep(sideSec)
      
     
      # the sequence is computed at 1 Hz frequency
      r.sleep()
    
    # at this point, either the goal has been achieved (success==true)
    # or the client preempted the goal (success==false)
    # If success, then we publish the final result
    # If not success, we do not publish anything in the result
    if success:
      self._result.result = (sideSec*4) + (turnSec*4)
      rospy.loginfo('Succeeded - Move In Square!')
      rospy.loginfo('Total Time = '+str(self._result.result))
      self._as.set_succeeded(self._result)
      
      # stop the drone
      self._move.linear.x = 0.0
      self._move.angular.z = 0.0
      self.publish_once(self._move)
      
      # land the drone
      i = 0
      while not i == 3:
        self._land.publish(self._land_msg)
        rospy.loginfo('DRONE LANDING ...')
        time.sleep(1)
        i += 1
      
if __name__ == '__main__':
  rospy.init_node('move_drone_square')
  MoveSquareClass()
  rospy.spin()