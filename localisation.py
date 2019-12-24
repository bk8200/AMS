#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import *
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
import numpy as np


# from ams import wrapToPi
# from world import MTAG
# import graph_gen as graph
# import tf.transformations as tft

class Localisation(object):
    def __init__(self):
        self._tfBroadcaster = tf2_ros.TransformBroadcaster()

        self._subOdom = rospy.Subscriber('odom', Odometry, self._handleOdometry)
        self._subTag = rospy.Subscriber('tag', Int64, self._handleTag)
        self.tag = 0
        self.phi = 0
        self.q_0 = 0

        self.P = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.Q = np.array([[1, 0], [0, 1]])
        self.R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def _handleTag(self, msg):
        self.tag = msg.data
        print('Tag data: {}'.format(msg.data))

    def _handleOdometry(self, msg):
        D = 0.1207
        # x, y, phi = 0.0, 0.0, 0.0   #  pravo x, y in phi pozicijo dobimo iz handleTag?
        delta_d = msg.twist.twist.linear.x
        gama = msg.twist.twist.angular.z
        self.phi = self.phi + gama
        phi = self.phi

        A = np.array([[1, 0, -delta_d * sin(phi) * cos(gama)], [0, 1, delta_d * cos(phi) * cos(gama)], [0, 0, 1]])
        F = np.array([[cos(phi) * cos(gama), -delta_d * cos(phi) * sin(gama)],
                      [sin(phi) * cos(gama), -delta_d * sin(phi) * sin(gama)],
                      [1 / 0.125 * sin(gama), delta_d / 0.125 * cos(gama)]])

        A_T = np.transpose(A)
        F_T = np.transpose(F)

        #### Predikcija ####

        # Odometrija
        dq_0 = np.array([delta_d * cos(gama) * cos(phi)], [delta_d * cos(gama) * sin(phi)], [delta_d * sin(gama) / D])
        self.q = self.q + delta_d * dq_0
        q = self.q

        x = A * q
        P = A * self.P * A_T + F * self.Q * F_T

        #### Korekcija ####
        K1 = np.dot(self.P, self.C)
        K2 = np.inv(
            np.dot(self.C, np.dot(self.P, self.C)) + self.R)  # preveri če je množenje matrik v pravilnem vrstnem redu
        K = np.dot(K1, K2)

        # Zadnji prebran tag
        z = self.tag  # Preveri ali vsebuje samo ID taga ali tudi pozicijo taga

        x_ocena = (z - x)
        x = x + np.dot(K, x_ocena)
        self.P = self.P - np.dot(self.K, np.dot(self.C, self.P))

        # '''
        trans = TransformStamped()
        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = 'world'
        trans.child_frame_id = 'NS/estimate'
        trans.transform.translation.x = x
        trans.transform.translation.y = y
        trans.transform.rotation.z = sin(phi / 2.0)
        trans.transform.rotation.w = cos(phi / 2.0)
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
