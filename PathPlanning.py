#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import *
from graph_gen import tagMap

class PathPlanning(object):
  def __init__(self):
    #Map structure {nodeID:  [leftID, leftDistance, rightID, rightDistance],}
    self.map =  tagMap
    nodeId = 2
   # print(nodeId, self.map[nodeId])
    #print 'Left path from the node {} leads to the node {}, distance of the path is {}'.format( nodeId, self.map[nodeId][0], self.map[nodeId][1])
    #print 'Right path from the node {} leads to the node {}, distance of the path is {}'.format(nodeId, self.map[nodeId][2], self.map[nodeId][3])

  def findPath(self, startId, goalId):
    #TODO
     #Preverimo ce je element ze na open listi
     #reverimo ce je nodeId enak 
    #Sestavimo path
    i = 0
    path = []
    closedList = {}
    #init - dodamo prvega na open listo
    openList = {startId : [0, 0]}
    for n in range(1000):

      #POisce najmanjsi pathToHere v open list
      currentId = min(openList.items(), key=lambda x: x[1][1])[0]

      # Ce nismo dobili zadnjega

      if goalId != currentId:

        #Trenutnega damo na closed listo
        closedList[currentId] = openList[currentId]

        #Odstranimo trenutnega s open liste
        openList.pop(currentId)

        #Dodamo sosede od trenutnega na open listo
        #Pri tem izracunamo se razdaljo do predhonega + razdaljo dodanih
        for i in range(0,len(self.map.get(currentId))-1, 2):

          #Preverimo ce je element ze na open listi

          #Dodamo novega ali popravljenega na open listo
          openList[self.map[currentId][i]] = [currentId, #Tle dodamo nove Id na open listo
          self.map[currentId][i+1] + closedList[currentId][1]] #izracunamo PathToHere

      #Ce smo dobili zadnjega
      else:
        break
        
    #Sestavimo path
    trenutniID = goalId
    for n in range(len(closedList)):

      #for key,value in closedList.items():
      find(closedLIst.items(), key=lambda x: x[1][0])

      #
      #path[n] = 


    #TODO Implement A* algorithm here ...
    print 'Path from the node {} to the node {} is:'.format(startId, goalId)
    print path
    return path

if __name__ == '__main__':
  pathPlanning = PathPlanning()
  pathPlanning.findPath(2, 12)
