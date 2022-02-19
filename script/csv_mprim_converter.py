import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import copy
from math import pi

def convertCSVtoMPRIM(csvfilename,outputname):
  ########## data processing ##########
  #parameters
  resolution_m = 1
  num_headings = 16
  num_prim = 4
  num_samples = 10 
  totalprim = num_headings*num_prim
  #param: 1-3: x,y,angle
  num_params = 3 #x,y,theta
  states = {}
  key = [0,0,0,0]
  initial, final, cost, pID = 0 , 0 , 0 , 0
  csvr = []
  costs = [1,5]

  with open(csvfilename) as csv_file:
      csv_reader = csv.reader(csv_file, delimiter=',')
      # filtering empty rows
      for row in csv_reader:
        if (row[0] != ''):
          csvr.append(row)

      # creating states dictionary
      for r in range(len(csvr)):
        row = [float(j) for j in csvr[r]]
          #check if the current row is the startpose
        if (row[0] == 0):
          i = 0 #2d array iterator
          #finds the end state
          next = [float(k) for k in csvr[r+num_samples-1]]
          #initial and final angle are discretised angles ranging from 0 to 15 

          initial = round(8*row[3]/pi)
          final = round(8*next[3]/pi)
          if (initial == final):
            cost = costs[0]
            ## threshold of 100 is used to differentiate primID 0 and 1 -- needs to be changed if values change
            ## pID 2: increase angle, 3: decrease angle (for angles <= pi)
            if (next[1]**2 + next[2]**2 > 100):
              pID = 1
            else:
              pID = 0
          elif (initial < final):
            cost = costs[1]
            pID = 2
          else:
            cost = costs[1]
            pID = 3

          ##### Structure of key : [xi,yi,thetai,xf,yf,thetaf, initialangle, finalangle, cost, primID] ######
          key = tuple(row[1:4]+next[1:4]+[initial,final,cost,pID])
          states[key] = np.zeros((num_samples,num_params))

        states[key][i][0:3]= row[1:4]

        i += 1

  keys = list(states.keys()).copy()

  #reflecting about x axis 
  for key in keys:
    ##### Structure of key : [xi,yi,2: thetai, 3: xf, 4: yf, 5: thetaf, 6:initialangle, 7:finalangle, cost,  primID] ######
    newkey = list(key)
    if newkey[2] != 0:
      newkey[2] = 2*pi - newkey[2]
      newkey[6] = round(8*newkey[2]/pi)
    newkey[4] *= -1
    newkey[5] = 2*pi - newkey[5]
    newkey[7] = round(8*newkey[5]/pi)
    if (newkey[7] == 16):
      newkey[7] = 0
      newkey[9] = 2 #from 15 to 0
    elif (newkey[6] == 0 and newkey[7] == 15):
      newkey[9] = 3
    elif (newkey[6] == newkey[7]):
      if (newkey[3]**2 + newkey[4]**2 > 100):
        newkey[9] = 1
      else:
        newkey[9] = 0
    elif (newkey[6] < newkey[7]):
      newkey[9] = 2
    else:
      newkey[9] = 3

    newkey = tuple(newkey)
    if ( round(newkey[4]) != 0.0 ):
      states[newkey] = states[key].copy()
      states[newkey][:,1] *= -1
      states[newkey][:,2] = 2*pi - states[newkey][:,2]


  keys = list(states.keys()).copy()
  #reflecting about y axis 
  for key in keys:
    ##### Structure of key : [xi,yi,2: thetai, 3: xf, 4: yf, 5: thetaf, 6:initialangle, 7:finalangle, cost, primID] ######
    newkey = list(key)
    newkey[3] *= -1
    if (round(newkey[3]) == 0):
      continue
    if (newkey[5] < pi):
      newkey[2] = pi - newkey[2]
      newkey[5] = pi - newkey[5]
    elif (newkey[2] == 0 and newkey[5] > pi):
      newkey[2] = pi
      newkey[5] = 9*pi/8
    else:
      newkey[2] = 3*pi - newkey[2]
      newkey[5] = 3*pi - newkey[5]
    newkey[6] = round(8*newkey[2]/pi)
    newkey[7] = round(8*newkey[5]/pi)

    if (newkey[6] == newkey[7]):
      if (newkey[3]**2 + newkey[4]**2 > 100):
        newkey[9] = 1
      else:
        newkey[9] = 0
    elif (newkey[6] < newkey[7]):
      newkey[9] = 2
    else:
      newkey[9] = 3

    newkey = tuple(newkey)
    states[newkey] = states[key].copy()
    states[newkey][:,0] *= -1

    if (states[newkey][0][2] <  pi):
        states[newkey][:,2] = pi - states[newkey][:,2]
    else:
        states[newkey][:,2] = 3*pi - states[newkey][:,2]


  keys = list(states.keys()).copy()
  if (len(keys) != totalprim):
    raise Exception("Data processing errors")

  #possible alternative to generate PRIM ID: to create another dictionary

  # scaling to resolution
  for key in states.keys():
    states[key][:,0:2] = states[key][:,0:2] * resolution_m
  ########## end of data processing ##########

  ########## visualisation ##########
  # data visualisation
  fig, ax = plt.subplots(figsize=(10, 8))
  colors = ['r','g','b','c']
  for key in states.keys():
    plt.plot(states[key][:,0], states[key][:,1], c = colors[key[6]%4])

  # Major ticks every 20, minor ticks every 5
  minor_ticks = np.arange(-1, 1, 0.025)
  major_ticks = np.arange(-1, 1, 0.125)
  ax.set_xticks(major_ticks)
  ax.set_xticks(minor_ticks, minor=True)
  ax.set_yticks(major_ticks)
  ax.set_yticks(minor_ticks, minor=True)
  ax.grid(which='both')
  # Or if you want different settings for the grids:
  ax.grid(which='minor', alpha=0.2)
  ax.grid(which='major', alpha=0.5)
  plt.show()
  ########## end of visualisation ##########

  ########## sorting keys ##########
  #sort dictionary keys to print into output file
  sortedkey = []
  helper = {}
  minangle = 100
  bubble = 10

  for key in states.keys():
    cc = 0
    if sortedkey:
      for comp in sortedkey:
        ##### Structure of key : [xi,yi,2: thetai, 3: xf, 4: yf, 5: thetaf, 6:initialangle, 7:finalangle, cost, primID] ######  
        if comp[6] > key[6]:
          sortedkey.insert(cc,key)
          break
        if (cc == len(sortedkey)-1):
          sortedkey.append(key)
          break
        cc+=1
    else: 
      sortedkey.append(key)
  
  for i in range(bubble):
    for j in range(len(sortedkey)-1):
      if sortedkey[j][6] == sortedkey[j+1][6]:
        if sortedkey[j][9] > sortedkey[j+1][9]:
          store = sortedkey.pop(j)
          sortedkey.insert(j+1,store)

  
  ########## end ofsorting keys ##########

  ########## print mprim file##########
    # output file
  f = open(outputname, 'w+')
  f.write("resolution_m: " + format(resolution_m,'.6f') +"\n")
  f.write("timeResolution: 0.100000\n")
  f.write("numberofangles: " + str(num_headings) +"\n")
  f.write("totalnumberofprimitives: " +str(totalprim) + "\n")
  primID = 0
  angleID = 0
  for key in sortedkey:
      ##### Structure of key : [xi,yi,2: thetai, 3: xf, 4: yf, 5: thetaf, 6:initialangle, 7:finalangle, 8:cost,  9:primID] ######
    f.write("primID: " + str(key[9]) +"\n")
    f.write("startangle_c: " + str(key[6]) +"\n")
    f.write("endpose_c: " +str(int(key[3])) +" "  + str(int(key[4])) + " " + str(key[7]) + "\n")
    f.write("additionalactioncostmult: " + str(key[8]) + "\n")
    f.write("intermediateposes: " +str(num_samples) + "\n")
    for row in states[key]:
      f.write(format(row[0],'.4f') + " " + format(row[1],'.4f') + " " + format(row[2],'.4f') + "\n")
  f.close()
  ########## end of print mprim file##########
  return sortedkey,states

## Visualisation is used to visualise the populaton of primitives in the search space
def visualisation(insortedkey, instates,depth):
  ########## visualisation ##########
  resolution_m = 1
  states = copy.deepcopy(instates)
  sortedkey = copy.deepcopy(insortedkey)
  # data visualisation
  for d in range(depth):
    keys = list(sortedkey).copy()
    for key1 in keys:
      key1 = list(key1)
      count = 0
      for key2 in keys:
        key2 = list(key2)
        if key1[7] == key2[6]:
          #print("key1: ", key1)
          #print("ley2: ", key2)
          newkey = key2.copy()
          newkey[0] += key1[3]
          newkey[1] += key1[4]
          newkey[3] += key1[3]
          newkey[4] += key1[4]
          newkey = tuple(newkey)
          #print(newkey)
          tempstates = states[tuple(key2)].copy()
          #print(tempstates)
          tempstates[:,0] += key1[3]*resolution_m
          tempstates[:,1] += key1[4]*resolution_m
          #print(tempstates)
          states[newkey] = tempstates
          sortedkey.append(newkey)
          count += 1
        if (count == 4):
          break
  fig, ax = plt.subplots(figsize=(10, 8))
  colors = ['r','g','b','c']
  for key in states.keys():
    plt.plot(states[key][:,0]/resolution_m, states[key][:,1]/resolution_m, c = colors[key[6]%4])
    plt.plot(key[0]*resolution_m,key[1]*resolution_m, c= 'black', marker ='o')
    plt.plot(key[3]*resolution_m,key[4]*resolution_m, c= 'black', marker ='o')
  # IMPORTANT, CHANGE THE RANGE OF X AND Y HERE AS NEEDED
  minor_ticks = np.arange(-50, 50, 1)
  major_ticks = np.arange(-50, 50, 5)
  ax.set_xticks(major_ticks)
  ax.set_xticks(minor_ticks, minor=True)
  ax.set_yticks(major_ticks)
  ax.set_yticks(minor_ticks, minor=True)
  ax.grid(which='both')
  # Or if you want different settings for the grids:
  ax.grid(which='minor', alpha=0.2)
  ax.grid(which='major', alpha=0.5)
  plt.show()
  ########## end of visualisation ##########
  return states

#File path is assuming python script is run in the parent repository
print("Type csv file name and press enter")
csvfilename = "csv/"+str(input())+".csv"
print("Type desired mprim file name and press enter")
outputfilename = "mprim/"+str(input())+".mprim"
skey, states = convertCSVtoMPRIM(csvfilename,outputfilename)
v = visualisation(skey,states,2)