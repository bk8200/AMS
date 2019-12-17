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
    print('findPath')
    #TODO
     #Preverimo ce je element ze na open listi
     #reverimo ce je nodeId enak 
    #Sestavimo path
    
    
    i = 0
    path = []
    closedList = {}
    #init - dodamo prvega na open listo
    openList = {startId : [0, 0]}
    while len(openList.items()) > 0:
      
      
      #Poisce najmanjsi pathToHere v open list
      currentId = min(openList.items(), key=lambda x: x[1][1])[0]

      # Ce nismo dobili zadnjega

      if openList.get(goalId,False):
        #Ce smo dobili zadnjega
        break
        
      #Trenutnega damo na closed listovalue
      closedList[currentId] = openList[currentId]

      #Odstranimo trenutnega z open liste
      openList.pop(currentId)

      #Dodamo sosede od trenutnega na open listo
      #Pri tem izracunamo se razdaljo do predhonega + razdaljo dodanih
      for i in range(0,len(self.map.get(currentId))-1, 2):

        if self.map[currentId][i] == 0:
          continue
        
        cenaDoSoseda = self.map[currentId][i+1]
        cenaTrenutnega =  closedList[currentId][1]
        #Ce je ze na closed listi
        if self.map[currentId][i] in closedList.keys():
          cenaZeNaClosed = (closedList.get(self.map[currentId][i])[1])
          if cenaZeNaClosed > (cenaDoSoseda + cenaTrenutnega):
            closedList.pop(self.map[currentId][i])
            openList[self.map[currentId][i]] = [currentId, #Tle dodamo nove Id na open listo
            cenaDoSoseda + cenaTrenutnega] #izracunamo PathToHere

        #Preverimo ce je element ze na open listi
        elif self.map[currentId][i] in openList.keys():
          cenaZeNaOpen = (openList.get(self.map[currentId][i])[1])
          if cenaZeNaOpen > (cenaDoSoseda + cenaTrenutnega):
          #Dodamo novega ali popravljenega na open listo
            openList[self.map[currentId][i]] = [currentId, #Tle dodamo nove Id na open listo
            cenaDoSoseda + cenaTrenutnega] #izracunamo Patht[currentId][1]
            
        else:
          openList[self.map[currentId][i]] = [currentId, #Tle dodamo nove Id na open listo
            cenaDoSoseda + cenaTrenutnega] #izracunamo Patht[currentId][1]          

          




        
    #Sestavimo path
    if goalId in openList.keys():
      closedList[goalId] = openList[goalId]

      path.append(goalId)
      trenutniID = goalId
      while trenutniID != startId:

        for key, value in closedList.items():
          
          if trenutniID == key:
            tmp_value = value[0]
        
        path.append(tmp_value)
        trenutniID = tmp_value
            
            
      path.reverse()
            
        #trenutniID = trenutniKey

        #find(closedLIst.items(), key=lambda x: x[1][0])

        #
        #path[n] = 


      #TODO Implement A* algorithm here ...
      print 'Path from the node {} to the node {} is:'.format(startId, goalId)
      print path


    return path
    
  def generateActions(self, path):
    print('generateActions')
    actions = [] # A list of lists: [string(action), float(segmentLength), int(nextNodeID)]
    print(len(path))
    for i in range(len(path)-1):
      print(i)

      
      curr_ID = path[i]
      next_ID = path[i+1]

      pot_naprej = self.map[curr_ID]
      print(pot_naprej)
      if next_ID == pot_naprej[0]:
        action = 'levo'
        segmentLength = pot_naprej[1]
        print('levo')
        nextNodeID = next_ID
      elif next_ID == pot_naprej[2]:
        action = 'desno'
        segmentLength = pot_naprej[3]
        nextNodeID = next_ID
        print('desno')
      elif next_ID == pot_naprej[4]:
        action = 'sredina'
        segmentLength = pot_naprej[5]
        nextNodeID = next_ID
        print('sredina')

      print('actionspred')
      actions.append([action, float(segmentLength), int(nextNodeID)])
      print('actionspo')
      
    print('actions') 
    print actions
    
    
    return actions

if __name__ == '__main__':
  pathPlanning = PathPlanning()
  pathPlanning.findPath(139, 131)

