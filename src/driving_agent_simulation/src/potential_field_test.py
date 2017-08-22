#!/usr/bin/env python
from math import sqrt, pow
from sympy import *
from copy import deepcopy
import time


def potentialField():
	neighborPosesList = list([[-3,2],[4,0],[-5,-5]])
	if len(neighborPosesList) == 0:
		print("No obstacles detected.")
		return
	else:
		x = Symbol('x')
		y = Symbol('y')
		agent_x_pos = 0
		agent_y_pos = 0.00000001
		attrCloud = 0
		replCloud = 0
		covX = 0.3
		covY = 0.9
		heuristicGain = 1
		potCloud = 0
		goalPosX = 0
		goalPosY = 5
		for i in neighborPosesList:
			obsX = i[0]
			obsY = i[1]
			replCloud = replCloud + 1/( (sqrt((x-obsX)**2)/covX) + (sqrt((y-obsY)**2)/covY) )
		attrCloud = sqrt((goalPosX - x)**2 + (goalPosY - y)**2) * heuristicGain
		potCloud = attrCloud - replCloud
		dpdx = diff(potCloud, x)
		dpdy = diff(potCloud, y)
		dx = dpdx.subs({x:agent_x_pos, y:agent_y_pos})
		dy = dpdy.subs({x:agent_x_pos, y:agent_y_pos})
		print(dx, dy)


def betterPotentialField(crit_gap_x, crit_gap_y):

 	neighborsPosesDict = {"A":[3,2], "B":[2,2], "C":[1,1], "D":[5,4], "E":[7,3], "F":[6,2]}

	closedSet = {}
	if bool(neighborsPosesDict):		# if the obstacle list has vehicle data
		for rootKey in neighborsPosesDict:
			if rootKey in closedSet.keys():		# if the obstacle was used. if yes, then jump to the next obstacles.
				pass
			else:
				closedSet[rootKey] = neighborsPosesDict[rootKey]

				# freeSet shoud be as: every elements in obstacle list - elements in closedSet
				freeSet = deepcopy(neighborsPosesDict)
				for key in closedSet:
					freeSet.pop(key,None)

				# set up frontier list. Add rootkey to frontierSet.
				frontierSet = {}
				frontierSet[rootKey] = neighborsPosesDict[rootKey]

				# setup boundaries of obstacle bubbles.
				minX = neighborsPosesDict[rootKey][0]
				maxX = neighborsPosesDict[rootKey][0]
				minY = neighborsPosesDict[rootKey][1]
				maxY = neighborsPosesDict[rootKey][1]

				# if freeSet is an empty dict, which means all other obstacles are already checked.
				# then output gaussian function from only one unit.
				if not bool(freeSet):
					print(minX, maxX, minY, maxY)
				else:
					# Since frontierSet and freeSet dicts are get involved calculation in every procedures
					# in following while loop, it cannot be modified until all the is over. 
					# It's necessary to set two temporary dictionaries for them and update them at end of
					# each loop.
					# create temp_frontierSet that is equivalent to frontierSet.
					temp_frontierSet = deepcopy(frontierSet)

					# create temp_freeSet that is equivalent to freeSet.
					temp_freeSet = deepcopy(freeSet)

					# jump out of the loop if there's no frontiers,
					# which means there are no adjacent vehicles around.
					while bool(frontierSet):
						# Use every frontier obstacles to search every obetacles that haven't been checked.
						for frontierKey in frontierSet:
							for neighborKey in freeSet:
								deltaX = abs(frontierSet[frontierKey][0] - freeSet[neighborKey][0])
								deltaY = abs(frontierSet[frontierKey][1] - freeSet[neighborKey][1])

								# If one frontier is closed enough with one left node, it means that they can be
								# integrated into one bigger obstacles and generate one potential field.
								if deltaX <= crit_gap_x and deltaY <= crit_gap_y:

									# renew the boundary of new bigger potential field
									minX = min(minX, freeSet[neighborKey][0])
									maxX = max(maxX, freeSet[neighborKey][0])
									minY = min(minY, freeSet[neighborKey][1])
									maxY = max(maxY, freeSet[neighborKey][1])

									# add the neighbor obstacle into closedSet so we don't need to consider
									# it any more if it appears as a root later. 
									closedSet[neighborKey] = freeSet[neighborKey]
									# add the neighbor obstacle into frontierSet so we will keep searching
									# if it has its own neighbors.
									temp_frontierSet[neighborKey] = freeSet[neighborKey]
									# delete the neighbor obstacle from freeSet since it's already checked.
									temp_freeSet.pop(neighborKey, None)

							# if one frontier has done finding all its neighbor, delete it from frontierSet.
							temp_frontierSet.pop(frontierKey, None)

						# update the changed frontierSet and freeSet for next loop's use.
						frontierSet = deepcopy(temp_frontierSet)
						freeSet = deepcopy(temp_freeSet)

					# if there's no frontier obstacles any more, which means that there was no neighbors around
					# the integrated obstacles. then return the boundaries of the big obstacles.
					print(minX, maxX, minY, maxY)

	# If there is no neighbors available, then we don't need to concern repulsive potential functions coming
	# from obstacles.
	else:
		pass
		
if __name__ == '__main__':
  startTime = time.time()
  # potentialField()
  betterPotentialField(1.5,1.5)
  timeInterval = time.time()-startTime
  print(timeInterval)