#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from PathPlanning import PathPlanning

class Planning(object):
  def __init__(self):
    self._pp = PathPlanning()
    # Subscriber to the start and goal tag ID (the two values are separated by a comma).
    self._subStartGoal = rospy.Subscriber('start_goal_tag', String, self._handleStartGoal)
    # Publisher of the path, which is given as a list of comma separated tag IDs.
    self._pubPathTags = rospy.Publisher('path_tags', String, queue_size=10)

  def _handleStartGoal(self, msg):
    startGoal = msg.data.split(',')
    path = self._pp.findPath(int(startGoal[0]), int(startGoal[1]))
    msgPathTags = String()
    msgPathTags.data = ','.join(str(x) for x in path)
    self._pubPathTags.publish(msgPathTags)

if __name__ == '__main__':
  rospy.init_node('planning')
  p = Planning()
  rospy.spin()
