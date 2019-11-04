#!/usr/bin/env python
# :linenos: #

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion

class map_navigation():

  def __init__(self):

    # declare the coordinates of interest
    # rostopic echo /amcl_pose
    self.xpoint0 = -0.000287930479607
    self.ypoint0 = -2.27447287102
    self.zpoint0 = 0.500031344487
    self.wpoint0 = 0.86600730628

    self.xpoint1 = 2.87164651888
    self.ypoint1 = -1.50061627525
    self.zpoint1 = 0.974558534123
    self.wpoint1 = 0.224133138041

    self.xpoint2 = 0.685491605282
    self.ypoint2 = 0.292088881021
    self.zpoint2 = -0.864689013324
    self.wpoint2 = 0.502307585287

    self.xpoint3 = -0.217735469099
    self.ypoint3 = -1.49562535589
    self.zpoint3 = -0.268414112472
    self.wpoint3 = 0.963303619959

    self.goalReached = False

    # initiliaze
    rospy.init_node('map_navigation', anonymous=False)

    num = 0
    if (num == 0):

      self.goalReached = self.moveToGoal(self.xpoint0, self.ypoint0, self.zpoint0, self.wpoint0)
      num += 1

    if (num == 1):

      self.goalReached = self.moveToGoal(self.xpoint1, self.ypoint1, self.zpoint1, self.wpoint1)
      num += 1

    if (num == 2):

      self.goalReached = self.moveToGoal(self.xpoint2, self.ypoint2, self.zpoint2, self.wpoint2)
      num += 1

    if (num == 3):

      self.goalReached = self.moveToGoal(self.xpoint3, self.ypoint3, self.zpoint3, self.wpoint3)


  def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

  def moveToGoal(self,xGoal,yGoal,zGoal,wGoal):


      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)


      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")



      goal = MoveBaseGoal()


      goal.target_pose.header.frame_id = "map"


      goal.target_pose.header.stamp = rospy.Time.now()


      goal.target_pose.pose.position = Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation = Quaternion(0,0,zGoal,wGoal)


      rospy.loginfo("Sending goal location ...")

      ac.send_goal(goal)

      ac.wait_for_result(rospy.Duration(60))

      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("You have reached the destination")
              return True

      else:
              rospy.loginfo("The robot failed to reach the destination")
              return False

if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")


