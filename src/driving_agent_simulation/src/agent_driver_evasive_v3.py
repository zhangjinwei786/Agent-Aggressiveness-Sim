#!/usr/bin/env python

import rospy
import sys, time, datetime, csv
from copy import deepcopy
from numpy import matrix, sign, exp
from numpy.linalg import inv
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32
from math import sqrt, atan2, pi, tan


class agent_driver:

	def __init__(self, ns, agent_x_pos, agent_y_pos, frontVehiclesNum):
		self.ns = ns
		rospy.init_node('agent_driver', anonymous=True)

		# Setup publishers and subscribers
		# the format(ns) looks for the namespace in the ros parameter server
		rospy.Subscriber('/godSwitch_topic', Bool, self.switchUpdate)
		rospy.Subscriber('front_dist'.format(ns), Float32, self.distUpdate)
		rospy.Subscriber('vel'.format(ns), Twist, self.feedbackVelUpdate)
		self.pub_speed = rospy.Publisher('cmd_vel'.format(ns), Twist, queue_size = 2)

		# Set subscribers of all front vehicles' pose topics.
		# The number of subscribers depends on how many front vehicles it has.
		rospy.Subscriber('path'.format(self.ns), Path, self.agentPoseUpdate)
		for i in range (1,frontVehiclesNum + 1):
			rospy.Subscriber('/' + str(i) + '_front_car/pose', Pose, self.neighborsPosesUpdate, i)

		# TODO: God switch is the main control switch of whole simluation. The movement
		#		doesn't start until it turns on. Set switch off in default.
		self.godSwitch = False

		# TODO: dictionary that record and update neighbor/front vehicles' instant positions.
		#		In default it is set empty dictionaries
		self.neighborsPosesDict = {}

		# agent initial linear and angular position is based on parameters.
		self.agent_pos = [agent_x_pos, agent_y_pos]
		self.agent_angle = 0

		# initial last position: same as current position
		self.agent_last_pos = [agent_x_pos, agent_y_pos]

		# initial linear and angular velocity as 0.
		self.agent_x_vel = 0
		self.agent_z_vel = 0

		# initial desired speed as 0.
		self.agent_desire_x_vel = 0

		# instant speed from catvehivle as feedback.
		self.agent_x_vel_feedback = 0

		# initial front distance as 10 meters
		self.frontDist = 10

		# gradient vector as polar coordinate ststem. Initialize the magnitude and angle as 0.
		self.gradientLength = 0
		self.gradientAngle = 0

		# TODO: set two states for the vehicle: 
		#		0 -> stop state
		#		1 -> slow-speed state
		#		2 -> normal-speed state
		#		Starting at stop state
		self.vehicleState = 0

		# set a time stamp for checking time intervals
		self.lastMsg = time.time()

		# set a start flag, which let certain code run only once; set True in default.
		self.startFlag = True

		# set a stop flag , which calls callback function in rospy.on_shutdown() only
		# once (Usually the callback fucntion will be called many times when script
		# stops). Set False in dafault.
		# Leave stop flag for future use.
		self.stopFlag = False

		# error value of sttering wheel.
		self.steeringError = 0
		self.steeringError_last = 0

		# recording start date and time for recording data. Initializes as zero.
		self.startdateNTime = 0


	# *********** subscriber callbacks and publish functions ***************

	# Two callback functions to update car's instant position, neighbor cars'
	# instant positions
	def neighborsPosesUpdate(self, data, carNum):
		pose_Coord = [data.position.x, data.position.y, data.orientation.z]
		self.neighborsPosesDict[carNum] = pose_Coord

	def agentPoseUpdate(self, data):
		self.agent_pos = [data.poses[-1].pose.position.x, data.poses[-1].pose.position.y]
		self.agent_angle = data.poses[-1].pose.orientation.z

	# Update instant velocity as feedback.
	def feedbackVelUpdate(self, data):
		self.agent_x_vel_feedback = data.linear.x

	# Update if vehicle is moving or stop based on God switch message
	def switchUpdate(self, data):
		self.godSwitch = data.data
		if self.godSwitch == True:
			print('Simulation starts')
		else:
			print('Simulation pauses')

	def distUpdate(self, data):
		self.frontDist = data.data

	# Publish car's movement (linear and angular velocities to Gazebo)
	def speedPublisher(self):
		newVel = Twist()
		newVel.linear.x = self.agent_x_vel
		if abs(self.agent_z_vel) < 0.01:
			newVel.angular.z = 0
		else:
			newVel.angular.z = self.agent_z_vel
		self.pub_speed.publish(newVel)
		# print('agent speed: ' + str(self.agent_x_vel))
		# print('steering angle: ' + str(float(self.agent_z_vel*180/pi)))

 	# **********************************************************************

	# ************** potential field function helper functions *************

 	def potentialField_boundaries(self, detect_y_min, detect_y_max, bound_min, bound_max):

 		# initialize agent position, detect range and bounaries's place:
		agent_y_pos = self.agent_pos[1]
		detect_y_range = [detect_y_min, detect_y_max]
		boundaries = [bound_min, bound_max]
		
		# - initilize the sum of repulsive forces coming from boundaries as zero. 
		# Since the boundaries are straight and parallel with x-axis, so the potential
		# forces coming from wall is tangential, which only exerts on y-axis only.
		# - The sum of repulsive force on x-axis is zero. but I still leave it for being 
		# organized.
		repl_x = 0
		repl_y = 0

		# set covariances and gains of each bound. After many tries it is found out that 
		# it's better to change those factors instantly based on agent's position and
		# orientation.
		covBound_left = 0.7 * exp(abs(self.agent_angle*160/(boundaries[0]-agent_y_pos)))
		covBound_right = 0.7 * exp(abs(self.agent_angle*160/(boundaries[1]-agent_y_pos)))
		boundGain_gaussian_left = exp(-self.agent_angle*50/abs(boundaries[0]-agent_y_pos))
		boundGain_gaussian_right = exp(self.agent_angle*50/abs(boundaries[1]-agent_y_pos))

		# if left or right bound is inside of potential field range, 
		# add derivative of corresponding gausian potential function to total repulsive potential functions
		if boundaries[0]>=agent_y_pos+detect_y_range[0]: 	
			repl_y = repl_y + boundGain_gaussian_left*(exp(-0.5*(agent_y_pos-boundaries[0])/covBound_left*(agent_y_pos-boundaries[0]))/sqrt(covBound_left*2*pi))
		if boundaries[1]<=agent_y_pos+detect_y_range[1]:
			repl_y = repl_y + -1*boundGain_gaussian_right*(exp(-0.5*(agent_y_pos-boundaries[1])/covBound_right*(agent_y_pos-boundaries[1]))/sqrt(covBound_right*2*pi))

		# return the sum of repulsive potential forces from boundaries.
		return repl_x, repl_y

 	def mergePotentialField(self, crit_gap_x, crit_gap_y, detect_x_min, detect_x_max, detect_y_min, detect_y_max, left_bound, right_bound):
	 	# This part of function can achieve following functions:
		# -- If few (more than one) obstacles are closed enough
		#    (their distance on eith x axis and y axis are smaller than
		#    given critical gaps), those obstacels will merge together
		#    and create one potential functions with bigger size.
		# ---- It could avoid the situation that agent tried to go through
		#      narrow gap between vehicles and thus crash.
		# -- If the merged potential functions are close to boundaries, it
		#    will merge with bounaries
		# ---- It could avoid the situation that agent tried to go through
		#      narrow gap between vehicles and bounaries thus crash.

		count = 0
 		# - initilize the sum of repulsive forces coming from obstacles as zero. 
		repl_x = 0
		repl_y = 0

		closedSet = {}
		if bool(self.neighborsPosesDict):		# if the obstacle list has vehicle data
			for rootKey in self.neighborsPosesDict:
				if rootKey in closedSet.keys():		# if the obstacle was used. if yes, then jump to the next obstacles.
					pass
				else:
					closedSet[rootKey] = self.neighborsPosesDict[rootKey]

					# freeSet shoud be as: every elements in obstacle list - elements in closedSet
					freeSet = deepcopy(self.neighborsPosesDict)
					for key in closedSet:
						freeSet.pop(key,None)

					# set up frontier list. Add rootkey to frontierSet.
					frontierSet = {}
					frontierSet[rootKey] = self.neighborsPosesDict[rootKey]

					# setup boundaries of obstacle bubbles.
					minX = self.neighborsPosesDict[rootKey][0]
					maxX = self.neighborsPosesDict[rootKey][0]
					minY = self.neighborsPosesDict[rootKey][1]
					maxY = self.neighborsPosesDict[rootKey][1]

					# if freeSet is an empty dict, which means all other obstacles are already checked.
					# then output gaussian function from only one unit.
					if not bool(freeSet): 
						temp_repl_x, temp_repl_y = self.obs_gaussian(minX, maxX, minY, maxY, detect_x_min, detect_x_max, detect_y_min, detect_y_max, left_bound, right_bound)
						repl_x = repl_x +temp_repl_x
						repl_y = repl_y +temp_repl_y
						count += 1
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
						temp_repl_x, temp_repl_y = self.obs_gaussian(minX, maxX, minY, maxY, detect_x_min, detect_x_max, detect_y_min, detect_y_max, left_bound, right_bound)
						repl_x = repl_x +temp_repl_x
						repl_y = repl_y +temp_repl_y
						count += 1

		# If there is no neighbors available, then we don't need to concern repulsive potential functions coming
		# from obstacles.
		else:
			print("No front vehicles detected.")
		# print('there are ' + str(count) + ' gaussian functions.')
		return repl_x, repl_y

	def obs_gaussian(self, minX, maxX, minY, maxY, detect_x_min, detect_x_max, detect_y_min, detect_y_max, left_bound, right_bound):
		# TODO: 4 conditions: close to left bound, close to right bound, close to both, close to neither bound.
		agent_x_pos = self.agent_pos[0]
		agent_y_pos = self.agent_pos[1]
		unit_covX = 100
		unit_covY = 25
		lane_width = 4
		unit_gain = 70
		cov_ratio = unit_covX/unit_covY
		leftGap = minY - left_bound
		rightGap = right_bound - maxY


		if detect_x_min < 0.5*(maxX + minX) - agent_x_pos < detect_x_max and detect_y_min < 0.5*(maxY + minY) - agent_y_pos < detect_y_max:
			# close to left bound:
			if leftGap <= 5 and rightGap > 5:
				u = [0.5*(maxX + minX), left_bound]
				covY = unit_covY * (maxY - left_bound)
				covX = unit_covX + (maxX - minX) * unit_covX/(lane_width*cov_ratio)
				if covX < cov_ratio*covY:
					covX = cov_ratio*covY
			# close to right bound:
			elif leftGap > 5 and rightGap <= 5:
				u = [0.5*(maxX + minX), right_bound]
				covY = unit_covY * (right_bound - minY)
				covX = unit_covX + (maxX - minX) * unit_covX/(lane_width*cov_ratio) 
				if covX < cov_ratio*covY:
					covX = cov_ratio*covY
			# close to both bounds:
			elif leftGap <=5 and rightGap <= 5:
				u = [0.5*(maxX + minX), 0]
				covY = 100
				covX = unit_covX + (maxX - minX) * unit_covX/(lane_width*cov_ratio)
			# close to neither bounds:
			else:
				u = [0.5*(maxX + minX), 0.5*(maxY + minY)]
				covY = unit_covY + unit_covY*(maxY - minY)/lane_width
				covX = unit_covX + (maxX - minX) * unit_covX/(lane_width*cov_ratio)
				if covX < cov_ratio*covY:
					covX = cov_ratio*covY

			gain = unit_gain * sqrt((covX*covY)/(unit_covX*unit_covY))
			mvnpdf = gain * exp(-0.5*(-u[1] + agent_y_pos)**2/covY - 0.5*(-u[0] + agent_x_pos)**2/covX)/(2*pi*sqrt(covX*covY))
			# dirX = 0.25*(-2*u[0] + 2*agent_x_pos)*exp(-0.5*(-u[1] + agent_y_pos)**2/covY - 0.5*(-u[0] + agent_x_pos)**2/covX)/(pi*covX*sqrt(covX*covY))
			dirX = 0
			dirY = 0.25*(-2*u[1] + 2*agent_y_pos)*exp(-0.5*(-u[1] + agent_y_pos)**2/covY - 0.5*(-u[0] + agent_x_pos)**2/covX)/(pi*covY*sqrt(covX*covY))
			repl_x = mvnpdf * dirX / sqrt(dirX**2 + dirY**2)
			repl_y = mvnpdf * dirY / sqrt(dirX**2 + dirY**2)
			return repl_x, repl_y
		else:
			return 0, 0

	def potentialField(self):
		agent_x_pos = self.agent_pos[0]
		agent_y_pos = self.agent_pos[1]
		detect_x_range = [-10,70]
		detect_y_range = [-16,16]
		boundaries = [-8.0, 8.0]
		heuristicGain = 1
		attr_x = 0
		attr_y = 0
		repl_x = 0
		repl_y = 0

		temp_repl_x, temp_repl_y = self.potentialField_boundaries(detect_y_range[0], detect_y_range[1], boundaries[0], boundaries[1])
		repl_x = repl_x + temp_repl_x
		repl_y = repl_y + temp_repl_y

		temp_repl_x, temp_repl_y = self.mergePotentialField(10, 6, detect_x_range[0], detect_x_range[1], detect_y_range[0], detect_y_range[1], boundaries[0], boundaries[1])
		repl_x = repl_x + temp_repl_x
		repl_y = repl_y + temp_repl_y

		attr_x = heuristicGain

		dx = attr_x + repl_x
		dy = attr_y + repl_y

		self.gradientLength = sqrt(dy**2 + dx**2)
		self.gradientAngle = atan2(dy, dx)
		# print(dx, dy, self.vehicleState)
		# print(self.gradientAngle*57.3, self.vehicleState)


	# ************* linear speed controller helper functions *************


	# Use potential field's gradient to decide which state should be switched. 
	# 1. If there has space to pass agent (which means that the magnitude of gradient vector
	#     is big enough), it approaches or keeps a high speed.
	# 2. If there's no space to pass agent (the agent was placed in the saddle point and
	#     magnitude of vector is small), it approaches or keeps a low speed.
	# 3. If the dradient is very cloes to obstables, which the gradient will become big enough
	#    and the direction is negative, then the speed will change to zero.

	# 		== == == == == cases design == == == == == ==
	# 1. stable state (as 0): vehicle runing be in stable.
	# 2. low speed state (as 1): vehicle running in a low speed
	# 2. fast speed state (as 2): vehicle running in high speed.

	def speedController(self, low_speed, high_speed, gradLenThresh):
		if self.frontDist >= gradLenThresh and -pi/8<self.agent_angle<pi/8:
			self.agent_desire_x_vel = high_speed
			self.vehicleState = 2
		elif self.frontDist >= 13:
			self.agent_desire_x_vel = low_speed
			self.vehicleState = 1
		else:
			# self.agent_desire_x_vel = 0
			self.agent_x_vel = 0
			self.vehicleState = 0

	# P controller: to avoid accelerating too fast and drifting.
	# Known current speed and desire speed, the car can accelerate/decelerate to reach
	# the speed in a certain value.
	# Small error can also be smoothed by setting tolerance value.
	def P_Controller(self, accel, tolerance):
		accel = abs(accel)
		error = self.agent_desire_x_vel - self.agent_x_vel_feedback
		timeStamp = time.time() - self.lastMsg
		if abs(error) > tolerance:
			self.agent_x_vel = self.agent_x_vel_feedback + sign(error) * accel * timeStamp
		else:
			self.agent_x_vel = self.agent_desire_x_vel

	# steering controller
	def steeringCtrl(self, high_speed, highLimit, low_speed, lowLimit):
		self.steeringError = self.gradientAngle - self.agent_angle
		'''
		if self.vehicleState == 1:
			limit = lowLimit
		elif self.vehicleState == 2:
			limit = highLimit
		else:
			limit = 0.1
		'''
		if self.agent_x_vel <= low_speed:
			limit = lowLimit
		elif self.agent_x_vel <= high_speed:
			limit = lowLimit - (lowLimit - highLimit)/(high_speed - low_speed)
		else:
			limit = highLimit
		PD_output = 0.8 * self.steeringError + 0 * (self.steeringError - self.steeringError_last)/(time.time() - self.lastMsg)
		if abs(PD_output) <= limit:
			self.agent_z_vel = PD_output
		else:
			self.agent_z_vel = sign(PD_output) * limit

	# Update time stamp for every loop.
	def recordLastMsg(self):
		# print("run time: " + str(time.time() - self.lastMsg))
		self.lastMsg = time.time()

	# ********************** Recorder functions *******************************
	# The trajectory's recording is written in .csv file.

	def configCSV(self, frontVehiclesNum):
		self.startdateNTime = str(datetime.datetime.now())
		with open("/home/wavelab01/Documents/" + self.startdateNTime + '.csv', 'wb') as csvfile:
			filewriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			initialMark = ['Time', 'agent_x', 'agent_y', 'agent_theta']
			for n in range(1, frontVehiclesNum+1):
				initialMark.append(str(n)+"_x")
				initialMark.append(str(n)+"_y")
				initialMark.append(str(n)+"_theta")
			filewriter.writerow(initialMark)

	def writeData(self):
		with open("/home/wavelab01/Documents/" + self.startdateNTime + '.csv', 'a') as csvfile:
			filewriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			dataInRow = [str(time.time()), str(self.agent_pos[0]), str(self.agent_pos[1]), str(self.agent_angle)]
			for n in range(1, len(self.neighborsPosesDict)+1):
				xPos = self.neighborsPosesDict[n][0]
				yPos = self.neighborsPosesDict[n][1]
				zAngle = self.neighborsPosesDict[n][2]
				dataInRow.append(xPos)
				dataInRow.append(yPos)
				dataInRow.append(zAngle)
			filewriter.writerow(dataInRow)


def main(argv):

    # we eventually get the ns (namespace) from the ROS parameter server for this node
	ns = sys.argv[1]
	init_pos_x = float(sys.argv[2])
	init_pos_y = float(sys.argv[3])
	frontVehiclesNum = int(sys.argv[4])
	rospy.loginfo('Agent Driver: ' + ns + ' successfully set.')
	rospy.loginfo('Initial position set in random: ' + '[' + str(init_pos_x) + ',' + str(init_pos_y) + ']')
	node = agent_driver(ns, init_pos_x, init_pos_y, frontVehiclesNum)
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if node.startFlag:
			node.configCSV(frontVehiclesNum)
			node.startFlag = False
			while not node.godSwitch:
				print('Waiting for start...')
				rospy.sleep(1)
		if node.godSwitch:
			node.potentialField()
			node.steeringCtrl(12, 0.17, 7, 0.25)	# (high_speed, highLimit, low_speed, lowLimit)
			node.speedController(8, 12, 15) 		# (low-speed, high-speed, threshold of gradient magnitude)
			node.P_Controller(3, 0.1)			# (acceleration, tolerance)
			node.speedPublisher()
			node.writeData()
		else:
			# reset vehicleState as slow-speed mode, and set desire speed as zero.
			# in this way the vehicle will slow down and stop instead of sudden stop
			# and drift.
			node.agent_desire_x_vel = 0
			node.P_Controller(12, 0.05)
			node.speedPublisher()
		node.recordLastMsg()
		rate.sleep()



if __name__ == '__main__':
	try:
		main(sys.argv)
	except rospy.ROSInterruptException:
		pass