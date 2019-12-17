#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import *
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
import numpy as np
#from ams import wrapToPi
#from world import MTAG
#import graph_gen as graph
#import tf.transformations as tft

class Localisation(object):
  def __init__(self):
    self._tfBroadcaster = tf2_ros.TransformBroadcaster()
    
    self._subOdom = rospy.Subscriber('odom', Odometry, self._handleOdometry)
    self._subTag = rospy.Subscriber('tag', Int64, self._handleTag)
    self.Q = np.matrix('1 0; 0 1')
    self.R = np.matrix('1 0 0; 0 1 0; 0 0 1')
    self.P = np.matrix('10 0 0; 0 10 0; 0 0 10')
    
    
  def _handleTag(self, msg):
    print('Tag data: {}'.format(msg.data))
    
  def _handleOdometry(self, msg):      
    #TODO Implement localisation algorithm ...
    x, y, phi = 0.0, 0.0, 0.0
    D = 0.1207
    
    delta_d = msg.twist.twist.linear.x
    gama = msg.twist.twist.angular.z 
    
    dq_0 = np.matrix('delta_d * cos(gama) * cos(phi); delta_d * cos(gama) * sin(phi); delta_d * sin(gama) / D;')
    
    # Odometrija
    q_1 = q_0 + delta_d * dq_0
    
    
    A_curr = np.matrix('1 0 -delta_d*sin(phi)*cos(gama);
                        0 1  delta_d*cos(phi)*cos(gama);
                        0 0                           1')
                        
    F_curr = np.matrix('cos(gama)*cos(phi) -delta_d*sin(gama)*cos(phi);
                        cos(gama)*sin(phi) -delta_d*sin(gama)*sin(phi);
                        sin(gama)/D         delta_d*cos(gama)/D')
                        
    C = np.eye(3)
    CT = np.transpose(C)
    
    
    # Predikcija
    
    Pk_k1 = A_curr * Pk1_k1 * np.transpose(A_curr) + F_curr * self.Q * np.transpose(F_curr)
    
    
    # Korekcija
    K = Pk_k1 * CT * np.inv(C * Pk_k1 * CT + R)
    
    q_1_ocena = q_0_ocena + K * (z - z_ocena)
    
    Pkk = Pk_k1 - K * C * Pk_k1
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    #'''
    trans = TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'world'
    trans.child_frame_id = 'NS/estimate'
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.rotation.z = sin(phi/2.0)
    trans.transform.rotation.w = cos(phi/2.0)
    self._tfBroadcaster.sendTransform(trans)
    '''
    vecTodom2wmr = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    vecQodom2wmr = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    vecTworld2wmr = np.array([x, y, 0.0])
    vecQworld2wmr = np.array([0.0, 0.0, sin(phi/2.0), cos(phi/2.0)])
    vecQworld2odom = tft.quaternion_multiply(vecQworld2wmr, tft.quaternion_conjugate(vecQodom2wmr))
    matRworld2odom = tft.quaternion_matrix(vecQworld2odom)[0:3,0:3]
    vecTworld2odom = vecTworld2wmr - matRworld2odom.dot(vecTodom2wmr)
    msgQworld2odom = Quaternion(*vecQworld2odom)
    msgTworld2odom = Point(*vecTworld2odom)
    
    trans2 = TransformStamped()
    trans2.header.stamp = msg.header.stamp
    trans2.header.frame_id = 'world'
    trans2.child_frame_id = 'NS/odom'
    trans2.transform.translation = msgTworld2odom
    trans2.transform.rotation = msgQworld2odom
    
    transMsg = TransformStamped()
    transMsg.header = msg.header
    transMsg.child_frame_id = msg.child_frame_id
    transMsg.transform.translation = msg.pose.pose.position
    transMsg.transform.rotation = msg.pose.pose.orientation
    
    self._tfBroadcaster.sendTransform([trans2, transMsg])
    #'''
    
if __name__ == '__main__':
  rospy.init_node('localisation')
  localisation = Localisation()
  rospy.spin()
