#!/usr/bin/env python

import rospy, sys, os
from random import shuffle
from multiprocessing import Pool


def placingVehicles(vehicleInfoList):
	vehicleNum = vehicleInfoList[0]
	robot_xPos = vehicleInfoList[1][0]
	robot_yPos = vehicleInfoList[1][1]
	numOfVehicles = int(sys.argv[1:][0])
	if vehicleNum == -1:
		robot_name = 'agent_car'
		rospy.loginfo(robot_name + 'position is at: ' + str(robot_xPos) + ',' + str(robot_yPos))
		os.system('roslaunch driving_agent_simulation load_vehicle.launch vehicle_name:=' + robot_name + ' init_pos_x:=' + str(robot_xPos) + ' init_pos_y:=' + str(robot_yPos) + ' is_agent:=true' + ' front_vehicle_number:=' + str(numOfVehicles))
	else:
		robot_name = str(vehicleNum)+'_front_car'
		rospy.loginfo(robot_name + 'position is at: ' + str(robot_xPos) + ',' + str(robot_yPos))
		os.system('roslaunch driving_agent_simulation load_vehicle.launch vehicle_name:=' + robot_name + ' init_pos_x:=' + str(robot_xPos) + ' init_pos_y:=' + str(robot_yPos) + ' is_agent:=false')

def run(frontVehiclesNum):

	# initializing the node, type of argv, and threads
	frontPlacesList = []
	agentPlacesList = []
	frontVehiclesNum = int(frontVehiclesNum[0])
	rospy.init_node('vehicle_assign', anonymous=True)
	p = Pool(11)

	# To limit number of vehicles from 1 to 10. 
	# Otherwise it reports error and the program stops.
	if frontVehiclesNum >= 10:
		rospy.logerr('For simulation performance, the number of front vehicles should be no more than 10.')
		return
	elif frontVehiclesNum < 0:
		rospy.logerr('The number of front vehicles cannot be less than zero.')
		return
	
	# setting up a list of front vehicle's possible initialize positions, shown in [x, y],
	# then shuffle the list to make it random
	for xPos in range(25, 76, 25):
		for yPos in range(-6, 7, 4):
			frontPlacesList.append([xPos,yPos])
	shuffle(frontPlacesList)
	'''
	#frontPlacesList.append([25,-2])
	#frontPlacesList.append([50,-2])
	#frontPlacesList.append([75,-2])
	# frontPlacesList.append([50,2])
	# frontPlacesList.append([75,6])
	'''
	# setting up a list of agent vehicle's possible initialize positions, shown in [x, y],
	# then shuffle the list to make it random
	xPos = 0
	for yPos in range(-6, 7, 4):
		agentPlacesList.append([xPos,yPos])
	shuffle(agentPlacesList)
	'''
	agentPlacesList.append([0,-2])
	'''
	# packing every vehicle numbers and position informarion to a list
	posList = []
		# packing front vehicle's info
	for vehicleNum in range(0, frontVehiclesNum):
		posList.append([vehicleNum+1, frontPlacesList[vehicleNum]])
		# packing agent vehicle's info
	posList.append([-1, agentPlacesList[0]])
	# print(posList)

	p.map(placingVehicles, posList)


if __name__ == '__main__':
    try:
        run(sys.argv[1:])
    except rospy.ROSInterruptException:
		pass