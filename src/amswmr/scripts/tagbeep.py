#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int64
import subprocess
import rospkg
import os

class TagBeep(object):
  def __init__(self):
    rp = rospkg.RosPack()
    self._path = os.path.join(rp.get_path('amswmr'), 'data', 'complete.oga')
    print(self._path)
    self._subTag = rospy.Subscriber('tag', Int64, self._handleTag)

  def _handleTag(self, msg):
    print('Tag data: {}'.format(msg.data))
    subprocess.call(['paplay', self._path])

if __name__ == '__main__':
    rospy.init_node('tag_beep')
    tagMapping = TagBeep()
    rospy.spin()
