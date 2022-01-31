#!/usr/bin/env python2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys
import time
import random

actions = {'U': (1,0), 'D': (-1,0), 'L': (0,-1), 'R': (0,1)}
pos = [8, 5]

def bayes(map, direction):
       #Change ALL values in map to be what every possible move is.
       # Bottom code ok but may need to be expanded to include every point.
       # Some positions will be changed 2ce in one iteration so... you need to combine the values.
       # #And is multiplication and or is addition 
       '''
       In this seciton I need to create a new array of same size as map.
       Then loop through my old map array to calculate the probabilties of taking a move from each position.
       Then taking that probabilty and adding it to the probability in my new array.
       '''
       change = actions[direction]
       n = map.shape
       a_map = np.zeros_like(map)
       for x in range(n[0]):
              for y in range(n[1]):
                     #Three potential edge cases No edge, 1 off edge, On edge
                     u_bnd = map.shape
                     move = [x + change[0], y + change[1]]
                     farMove = [x + 2*change[0], y + 2*change[1]]
                     if(0 > move[0] or 0 > move[1] or u_bnd[0] <= move[0] or u_bnd[1] <= move[1]): #ON EDGE
                            #If the move spot is out of bounds then both farMove and move are out of bounds
                            a_map[x,y] += (map[x,y] * 1) 
                     else:
                            if(0 > farMove[0] or 0 > farMove[1] or u_bnd[0] <= farMove[0] or u_bnd[1] <= farMove[1]):#1 OFF EDGE
                                   #If this far spot is out of bounds then only move and the current position are within bounds
                                   a_map[x,y] += (map[x,y] * 0.2)
                                   a_map[move[0], move[1]] += (map[x,y] * 0.8)
                            else: #NO EDGE
                                   #All edge checks have been met and that means that we are not close to any walls and can use the normal probability.
                                   a_map[x,y] += (map[x,y] * 0.2)
                                   a_map[move[0], move[1]] += (map[x,y] * 0.6)
                                   a_map[farMove[0], farMove[1]] += (map[x,y] * 0.2)
       return a_map

def sumMap(map):
       n = map.shape
       total = 0.0
       for x in range(n[0]):
              for y in range(n[1]):
                     total+= map[x,y]
       print(total)

if __name__ == '__main__':
       map = np.zeros((20,10), np.float64)
       map[pos[0]][pos[1]] = 1
       moveList = list(sys.argv[1])
       fig, ax = plt.subplots()
       im = plt.imshow(map)
       # Create colorbar
       plt.clim(0, .12)
       plt.colorbar()
       plt.yticks(range(map.shape[0]))
       plt.xticks(range(map.shape[1]))
       ax.invert_yaxis()
       
       im.set_data(map)
       plt.pause(1)
       for dir in moveList:
              map = bayes(map, dir)
              im.set_data(map)
              plt.pause(1)
              
