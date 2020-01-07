#!/usr/bin/python
# -*- coding: utf-8 -*-
from ams import *
from wmr import Wmr
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int64
from math import cos, sin, atan2
from world import MTAG

def wrapToPi(kot):
  kot = atan2(sin(kot),cos(kot))
  return kot  


class WmrRos(Wmr):
  def __init__(self):
    Wmr.__init__(self)
    
    self._pubOdom = rospy.Publisher('odom', Odometry, queue_size=10)
    self._subCmdVel = rospy.Subscriber('cmd_vel', Twist, self._handleCmdVel)
    self.subTag = rospy.Subscriber('tag',Int64,self._handleTag)
    self._subActions = rospy.Subscriber('path_actions', String, self._handleActions)
    self.encoderLOld = 0.0
    self.encoderROld = 0.0
    self.flag = 0
    self.x = 0.0
    self.y = 0.0
    self.fi = 0.0
    self.old_error = 0.0
    self.gama = 0.0
    self.D = 0.1207
    self.tagId = 0
    self.currentIdx = 0
    self.prepotovana_pot = 0.0    
    self.sled_rob = 'levo'
    self.new_path  = 1
    self.actions = []
  
  def _handleCmdVel(self, msg):
    self.setVel(msg.linear.x, msg.angular.z)
    
  def _handleTag(self, msg):
    print(msg)
    self.tagId = MTAG.get(msg.data, -1)

    
  def _handleEncoders(self, left, right, heading):
    if self.flag == 0:
      self.flag = 1
      self.encoderLOld = left 
      self.encoderROld = right

    max_gama = 8191.0
    self.gama = - (heading - 5630.0) * 2.0 * pi / max_gama #3674.0
    
    self.gama = wrapToPi(self.gama)
    
    #print(str(180*self.gama/pi) + '   ' +str(heading))
    
    dl = -(left - self.encoderLOld)
    dr = right - self.encoderROld
    
    d = (dl + dr) / 2.0
    
    konst = 110000.0
    
    d = d / konst
    
    dx = d*cos(self.gama)*cos(self.fi)
    dy = d*cos(self.gama)*sin(self.fi)
    dfi = d*sin(self.gama) / self.D
    
    self.x = self.x + dx
    self.y = self.y + dy
    self.fi = self.fi + dfi

    self.prepotovana_pot = self.prepotovana_pot + sqrt(dx**2 + dy**2)
    #TODO Implement odometry
    
    self.encoderLOld = left
    self.encoderROld = right 
            
    #print(self.x, heading, 360*self.gama/(2*pi), d)       
    #print( heading)
    #print(self.x, self.y, 360*self.fi/(2*pi), 360*self.gama/(2*pi), heading)
    #print(left, right, heading, 360*self.gama/(2*pi))
    
    
    
    
    msg = Odometry()
#    msg.header.stamp = t
    msg.header.frame_id = 'jaka/odom'
    msg.child_frame_id = 'jaka/wmr'
    msg.pose.pose.position.x = self.x
    msg.pose.pose.position.y = self.y
    msg.pose.pose.orientation = phiToQuaternionMsg(self.fi)
    
    msg.twist.twist.linear.x = d # delta_d
    msg.twist.twist.angular.z = self.gama # gama kot / heading
    
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
    
    
    if self.actions != []:
      if self.currentIdx == 0:
        self.sled_rob = self.actions[self.currentIdx][0]
            
      if self.actions[self.currentIdx][2] > 100: # Imag
        if self.prepotovana_pot > self.actions[self.currentIdx][1]/1000.0:
          print('Imag reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0     
          #self.sled_rob = self.actions[self.currentIdx][0]
      elif self.actions[self.currentIdx][2] < 100 and self.actions[self.currentIdx][2] > 0 : # Real
        if self.tagId == self.actions[self.currentIdx][2]:
          print('Real reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0     
          #self.sled_rob = self.actions[self.currentIdx][0]
      #print(str(self.currentIdx) + ' ' + str(len(self.actions)))
      if self.currentIdx >= len(self.actions):
        self.actions = []
        self.currentIdx = 0
        print('Konec poti')
      else:
        self.sled_rob = self.actions[self.currentIdx][0]
    else:
      self.sled_rob = 0
    


  
    if self.sled_rob == 'levo':
      line_error = left;
      
      
    elif self.sled_rob == 'desno':
      line_error = right;

      
    if self.sled_rob == 'sredina':
      '''
      x_ref = 0.5
      y_ref = 0.5
      
      dist_to = sqrt((self.x -x_ref)**2+(self.y -y_ref)**2)
      
      fi_ref = atan2((y_ref-self.y),(x_ref-self.x))
      
      e_fi = wrapToPi(fi_ref - self.fi)
      
      K_fi = 1.0
      
      gama_ref = K_fi * e_fi
      
      K_gama = 2.0
      ws = K_gama * ( gama_ref - self.gama )
      
      
      Kv = 2.0
      vs = Kv * dist_to / cos(self.gama)
      
      self.setVel(vs, ws)

      
      
      
      
      
      
      '''
      x_ref = 0.5
      y_ref = 0.5
      v_max = 0.3
      Kw = 2.5
      Kgama = 4.0
      Kv = 2.0
      dist_min = 0.1
      dist_to = sqrt((self.x -x_ref)**2+(self.y -y_ref)**2)
      
      fi_ref = atan2((y_ref-self.y),(x_ref-self.x))
      
      e_fi = wrapToPi(fi_ref - self.fi)
      
      
      w = Kw * (e_fi)
      v = Kv * dist_to 
      
      if abs(v) > v_max:
        v = sign(v) * v_max
      
      
      vs = sqrt(w**2 * self.D**2 + v**2)
      gama_ref = atan2(w*self.D, v)
      
      e_gama = wrapToPi(gama_ref - self.gama)
      ws = Kgama * e_gama
      #print(str(180*e_fi/pi))
      
 
      brake = (dist_to/dist_min)**4
      if brake > 1:
        brake = 1
      
      #print(str(brake))
      
      
      
      ws = brake*ws
      vs = brake*vs*abs(cos(e_gama/2))
      
      #vs = v_max * vs / (abs(vs)+abs(ws))
      
      
      self.setVel(vs, ws)
           
     
      
      
      
      
      
      
    
    elif left == None or right == None:
      print('None')
      
    elif  self.sled_rob != 0:
      
      
      Kwp = 2
      
      Kvd = 0.5
      Kvp = 0.1
      
      
        
      ws = Kwp * line_error
      vs = Kvp #+ Kvd * (self.old_error - line_error)
    
      self.setVel(vs, ws)
    
      
      self.old_error = line_error
     
    
    #print('Leva, desna: ' + str(left) + ' ' + str(right) + ' Senzor: ' + str(sensors))
  def _handleActions(self, msg):
    print(msg.data)
    a = msg.data.split(';')
    if len(a):
      b = [x.split(',') for x in a]
      self.actions = [(x[0], float(x[1]), int(x[2])) for x in b if len(x) > 2] 
    if self.currentIdx == 0:
      self.sled_rob = self.actions[self.currentIdx][0]

    

    
    
    
    
        
        
  
  
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
