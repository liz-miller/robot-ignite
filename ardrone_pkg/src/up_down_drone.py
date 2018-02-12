#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_pkg.msg import ArdroneActionActionFeedback, ArdroneActionActionResult, ArdroneActionAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class UpDownDroneClass(object):
    
  # create messages that are used to publish feedback/result
  _feedback = ArdroneActionActionFeedback()
  _result = ArdroneActionActionResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("up_down_drone_as", ArdroneActionAction, self.goal_callback, False)
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
    
    command = goal.goal
    
    i = 0 
    for i in xrange(0,4):
      
      if self._as.is_preempt_requested() and not i > 3:
        rospy.loginfo('Cancelling move request')
        self._as.set_preempted()
        success = False
        self.ctrl_c = True
      
      #publish the move to the drone
      if command == 'up':
        i = 0
        while not i == 2:
          rospy.loginfo('Going up... Loop #: '+str(i+1))
          self._takeoff.publish(self._takeoff_msg)
          self._as.publish_feedback(self._feedback)
          i += 1
          time.sleep(1)
          command = "stay"
        
      if command == 'down':
        success = True
        rospy.loginfo('Going down...')
        self._land.publish(self._land_msg)
        self._as.publish_feedback(self._feedback)
      
      if command == 'stay':
        success = True
        rospy.loginfo('Staying at current pos')
        self._move.linear.z = 0.0
        self._move.angular.z = 1.0
        self.publish_once(self._move)
        time.sleep(10)
        command = "down"
      
      r.sleep()
      
      # at this point, either the goal has been achieved (success==true)
      # or the client preempted the goal (success==false)
      # If success, then we publish the final result
      # If not success, we do not publish anything in the result
      if success:
        self._result = Empty()
        self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('up_down_drone')
  UpDownDroneClass()
  rospy.spin()