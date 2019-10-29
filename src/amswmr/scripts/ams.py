#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import *
from numpy import sign
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Transform, TransformStamped

def wrapToPi(x):
  '''Wrap the angle to [-pi, pi]'''
  return x-floor((x+pi)/2.0/pi)*2.0*pi

def quaternionMsgToPhi(msg):
  '''Read the angle from Quaternion message'''
  return 2*asin(msg.z)*sign(msg.w)
  
def phiToQuaternionMsg(phi):
  '''Write the angle to Quaternion message'''
  msg = Quaternion()
  msg.w = cos(phi/2.0)
  msg.z = sin(phi/2.0)
  return msg
  
def msgToPose(msg):
  '''Get pose (x, y and phi) from Pose or Transform message
  
     Arguments:
       msg = Pose(), PoseStamped(), Transform() or TransformStamped() from geometry_msgs.msg
       
     Return: (x, y, phi)
  '''
  if isinstance(msg, PoseStamped):
    msg = msg.pose
  elif isinstance(msg, TransformStamped):
    msg = msg.transform
    
  if isinstance(msg, Pose):
    ans = (msg.position.x, msg.position.y, quaternionMsgToPhi(msg.orientation))
  elif isinstance(msg, Transform):
    ans = (msg.translation.x, msg.translation.y, quaternionMsgToPhi(msg.rotation))
  else:
    raise Exception('Unknown input type.')
  
  return ans

def poseToPoseMsg(x, y, phi=0.0):
  '''Write pose (x, y, phi) to Pose message'''
  msg = Pose()
  msg.position.x = x
  msg.position.y = y
  msg.orientation = phiToQuaternionMsg(phi)
  return msg
  
def poseToTransformMsg(x, y, phi=0.0):
  '''Write pose (x, y, phi) to Transform message'''
  msg = Transform()
  msg.translation.x = x
  msg.translation.y = y
  msg.rotation = phiToQuaternionMsg(phi)
  return msg
