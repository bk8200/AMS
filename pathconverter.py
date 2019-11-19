#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from graph import Graph

class PathConverter(object):
  def __init__(self):
    self._graph = Graph()
    
    self._lastStamp = None
      
    # Name of the world frame
    self._worldFrameId = rospy.get_param('~world_frame_id', 'world')
    # Publisher of the path.
    self._pubPath = rospy.Publisher('path', Path, queue_size=10)
    # Subscriber to the path, which is given as a list of comma separated tag IDs.
    self._subPathTags = rospy.Subscriber('path_tags', String, self._handlePath)

  def _handlePath(self, msg):
    path = msg.data.split(',')
    try:
      path = [int(x) for x in path]
    except ValueError:
      return
    
    if len(path):
      p = self._graph.getPath(path)
      msgPath = Path()
      msgPath.header.frame_id = self._worldFrameId
      msgPath.header.stamp = rospy.Time.now()
      for i in range(p.shape[1]):
        pose = PoseStamped()
        pose.pose.position.x = p[0,i]/1000.0
        pose.pose.position.y = p[1,i]/1000.0
        msgPath.poses.append(pose)
      self._pubPath.publish(msgPath)
      
if __name__ == '__main__':
  rospy.init_node('path_converter')
  pathConverter = PathConverter()
  rospy.spin()
