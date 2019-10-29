#!/usr/bin/python
# -*- coding: utf-8 -*-
from ams import *
from wmr import Wmr
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int64
from math import cos, sin




class WmrRos(Wmr):
  def __init__(self):
    Wmr.__init__(self)
    
    self._pubOdom = rospy.Publisher('odom', Odometry, queue_size=10)
    self._subCmdVel = rospy.Subscriber('cmd_vel', Twist, self._handleCmdVel)
    self.subTag = rospy.Subscriber('tag',Int64,self._handleTag)
    self.encoderLOld = 0.0
    self.encoderROld = 0.0
    self.flag = 0
    self.x = 0.0
    self.y = 0.0
    self.fi = 0.0
    self.old_error = 0.0
    
  def _handleCmdVel(self, msg):
    self.setVel(msg.linear.x, msg.angular.z)
    
  def _handleTag(self, msg):
    print(msg)
    
  def _handleEncoders(self, left, right, heading):
    if self.flag == 0:
      self.flag = 1
      self.encoderLOld = left 
      self.encoderROld = right

    max_gama = 8191.0
    gama = - (heading - 4320.0) * 2.0 * pi / max_gama
    
    dl = -(left - self.encoderLOld)
    dr = right - self.encoderROld
    
    d = (dl + dr) / 2.0
    
    konst = 110000.0
    D = 0.1207
    
    d = d / konst
    
    self.x = self.x + d*cos(gama)*cos(self.fi)
    self.y = self.y + d*cos(gama)*sin(self.fi)
    self.fi = self.fi + d*sin(gama) / D

    #TODO Implement odometry
    
    self.encoderLOld = left
    self.encoderROld = right 
            
    #print(self.x, heading, 360*gama/(2*pi), d)       
    #print( heading)
    #print(self.x, self.y, 360*self.fi/(2*pi), 360*gama/(2*pi), heading)
    #print(left, right, heading, 360*gama/(2*pi))
    
    
    
    
    msg = Odometry()
#    msg.header.stamp = t
    msg.header.frame_id = 'jaka/odom'
    msg.child_frame_id = 'jaka/wmr'
    msg.pose.pose.position.x = self.x
    msg.pose.pose.position.y = self.y
    msg.pose.pose.orientation = phiToQuaternionMsg(self.fi)
    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.angular.z = 0.0
    self._pubOdom.publish(msg)
  
  def _handleLineSensor(self, left, right, sensors):
    # None, če ga ne zazna črte
    #  self.setVel(v, w)
    #  self.setWheelVel(left, right)
    '''
    K = 0.1
    if left == None or right == None:
      print('None')
    else:
      line_error = (left + right) / 2

        
      if abs(line_error) < 0.1:
        base_speed = 0.05
      else:
        base_speed = 0.0
       
      vv = K*abs(line_error) + base_speed
      if line_error > 0:
        self.setWheelVel(-vv/2, -vv)
        print('Desno kolo:' + str(vv))
      else:
        self.setWheelVel(-vv,-vv/2)
        print('Levo kolo:' + str(vv))
        '''
        
    print(str(self.subTag))
    sled_rob = 1# levi rob -1, desni rob 1
    
    if sled_rob == -1:
      line_error = left;
      
      
    elif sled_rob == 1:
      line_error = right;
    
    if left == None or right == None:
      print('None')
    else:
        
      Kwp = 2
      
      Kvd = 0.5
      Kvp = 0.1
      
      
        
      ws = Kwp * line_error
      vs = Kvp #+ Kvd * (self.old_error - line_error)
    
      self.setVel(vs, ws)
    
      
      self.old_error = line_error
    
    
    #print('Leva, desna: ' + str(left) + ' ' + str(right) + ' Senzor: ' + str(sensors))
        
        
        
  
  
if __name__ == '__main__':
  try:
    rospy.init_node('wmr')

    with WmrRos() as wmr:
      wmr.startUp()
      rate = rospy.Rate(50)
      while not rospy.is_shutdown():
        wmr.updateSensors()
        rate.sleep()
  except KeyboardInterrupt:
    pass
