#!/usr/bin/env python

# Author: Jinwei Zhang

# This node is going to randomly assign the agent vehicles and certain numbers of 
# front vehicles into Gazebo environment, and then load their simulation program 
# and controller.
# It firstly creates two grids (shown in matrix), for saving all possible vehicles 
# positions for front vehicles and agent vehicle in a certain range. The boundaries
# (in x and y axis) and each grid's size can be customized.
# Then it enables each vehicle to randomly get possible positions from corresponding
# grid matrix, and load those vehicles in simulations.

import rospy, sys, os
from random import shuffle
from multiprocessing import Pool


def positionsSetup(frontVehiclesNum):
	# This function is to assign vehicles into certain positions in random.

	# Customizing the bound of grids and size of each grids.
	front_pos_bound_x = [25, 75]
	front_pos_gridSize_x = 25
	front_pos_bound_y = [-6, 6]
	front_pos_gridSize_y = 4

	agent_pos_bound_x = 0
	agent_pos_bound_y = [-6, 6]
	agent_pos_gridSize_y = 4


	# initializing the node and type of argv. 
	frontVehiclesNum = int(frontVehiclesNum[0])
	rospy.init_node('vehicle_assign', anonymous=True)

	# initialize all posible positions, shown in 2-dim matrices, for front car and agent cars.
	frontPlacesList = []
	agentPlacesList = []

	# To limit number of front vehicles from 0 to 10. 
	# Otherwise it reports error and then terminates program.
	if frontVehiclesNum >= 10:
		rospy.logerr('For simulation performance, the number of front vehicles should be no more than 10.')
		return
	elif frontVehiclesNum < 0:
		rospy.logerr('The number of front vehicles cannot be less than zero.')
		return
	
	# setting up a list of front vehicle's possible initial positions, shown in [x, y],
	# then shuffle the list to make it random
	for xPos in range(front_pos_bound_x[0], front_pos_bound_x[1]+1, front_pos_gridSize_x):
		for yPos in range(front_pos_bound_y[0], front_pos_bound_y[1]+1, front_pos_gridSize_y):
			frontPlacesList.append([xPos,yPos])
	shuffle(frontPlacesList)

	# setting up a list of agent vehicle's possible initial positions, shown in [x, y],
	# then shuffle the list to make it random
	xPos = agent_pos_bound_x
	for yPos in range(agent_pos_bound_y[0], agent_pos_bound_y[1]+1, agent_pos_gridSize_y):
		agentPlacesList.append([xPos,yPos])
	shuffle(agentPlacesList)

	# encoding every vehicle numbers and position information, and merge them into a list.
	# Each element's format is: [vehicle_number,[vehicle_pos_x, [vehicle_pos_x,]]
	# The reason of merging vehicle information into one variable is that it will be used 
	# for multithreading processing when loading vehicles and multithreading function can
	#  only have one variable.
	posList = []
	# encode front vehicle's info
	for vehicleNum in range(0, frontVehiclesNum):
		posList.append([vehicleNum+1, frontPlacesList[vehicleNum]])
		# encode agent vehicle's info. Agent vechicle's number is labelled as -1.
	posList.append([-1, agentPlacesList[0]])
	return posList


def loadingVehicles(vehicleInfoList):
	# This function decodes vehicle's information and load vehicles based on that.

	# decode the vehicle's information to certain vehicle's number and position.
	vehicleNum = vehicleInfoList[0]
	robot_xPos = vehicleInfoList[1][0]
	robot_yPos = vehicleInfoList[1][1]
	numOfVehicles = int(sys.argv[1:][0])

	# simulate Linux terminal and load corresponding vehicle's simulation stuff.
	if vehicleNum == -1:
		robot_name = 'agent_car'
		rospy.loginfo(robot_name + 'position is at: ' + str(robot_xPos) + ',' + str(robot_yPos))
		os.system('roslaunch driving_agent_simulation load_vehicle.launch vehicle_name:=' + robot_name + ' init_pos_x:=' + str(robot_xPos) + ' init_pos_y:=' + str(robot_yPos) + ' is_agent:=true' + ' front_vehicle_number:=' + str(numOfVehicles))
	else:
		robot_name = str(vehicleNum)+'_front_car'
		rospy.loginfo(robot_name + 'position is at: ' + str(robot_xPos) + ',' + str(robot_yPos))
		os.system('roslaunch driving_agent_simulation load_vehicle.launch vehicle_name:=' + robot_name + ' init_pos_x:=' + str(robot_xPos) + ' init_pos_y:=' + str(robot_yPos) + ' is_agent:=false')


def run(frontVehiclesNum):
	assigned_posList = positionsSetup(frontVehiclesNum)
	# I think the variable in Pool means how many multithreads, in maximum, can
	# be supported. It can also be adjusted based on computer's performance.
	p = Pool(11)
	p.map(loadingVehicles, assigned_posList)


if __name__ == '__main__':
    try:
        run(sys.argv[1:])
    except rospy.ROSInterruptException:
		pass