#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from PathPlanning import PathPlanning
from itertools import tee

'''
# Implement the following method in PathPlanning class: 
  def generateActions(self, path):
    actions = [] # A list of lists: [string(action), float(segmentLength), int(nextNodeID)]
    return actions
'''

'''
# To subscribe to actions in Wmrros class, add to the constructor:
    self._subActions = rospy.Subscriber('path_actions', String, self._handleActions)
# and implement the following function inside Wmrros class:
  def _handleActions(self, msg):
    print(msg.data)
    a = msg.data.split(';')
    if len(a):
      b = [x.split(',') for x in a]
      actions = [(x[0], float(x[1]), int(x[2])) for x in b if len(x) > 2]
'''

class Planning(object):
  def __init__(self):
    self._pp = PathPlanning()
    # Subscriber to the start and goal tag ID (the two values are separated by a comma).
    self._subStartGoal = rospy.Subscriber('start_goal_tag', String, self._handleStartGoal)
    # Publisher of the path, which is given as a list of comma separated tag IDs.
    self._pubPathTags = rospy.Publisher('path_tags', String, queue_size=10, latch=True)
    # Publisher of the actions, which are given as a semicolon separated list of actions.
    self._pubPathActions = rospy.Publisher('path_actions', String, queue_size=10, latch=True)

  def _handleStartGoal(self, msg):
    try:
      print('_handleStartGoal')
      startGoal = msg.data.split(',')
      
      path = self._pp.findPath(int(startGoal[0]), int(startGoal[1]))
      patha, pathb = tee(path)
      
      msgPathTags = String()
      msgPathTags.data = ','.join(str(x) for x in patha)
      if msgPathTags.data != '':
        self._pubPathTags.publish(msgPathTags)
      
      actions = self._pp.generateActions(list(pathb))
      msgPathActions = String()
      msgPathActions.data = ';'.join('{:s},{:f},{:d}'.format(*x) for x in actions)
      if msgPathActions.data != '':
        self._pubPathActions.publish(msgPathActions)
    except AttributeError:
      print('Error! Something went wrong :(')

if __name__ == '__main__':
  rospy.init_node('planning')
  p = Planning()
  rospy.spin()
