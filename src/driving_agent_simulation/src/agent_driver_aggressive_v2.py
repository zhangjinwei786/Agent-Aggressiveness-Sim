#!/usr/bin/env python

import rospy
import sys, time, datetime, csv
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
			rospy.Subscriber('/' + str(i) + '_front_car/front_dist', Float32, self.neighborsFrontDistUpdate, i)

		# TODO: God switch is the main control switch of whole simluation. The movement
		#		doesn't start until it turns on. Set switch off in default.
		self.godSwitch = False

		# TODO: dictionary that record and update neighbor/front vehicles' instant positions.
		#		In default it is set empty dictionaries
		self.neighborPosesDict = {}

		self.neighborFrontDistDict = {}

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

		# TODO: get gamma, omega, zeta values for coefficients of steering dynamics
		self.gamma = 0.5
		self.omega0 = 0.5
		self.zeta = 3

		# TODO: steering dynamics matrices and controller matrices
		self.A = matrix([])
		self.B = matrix([])
		self.C = matrix([])
		self.K = matrix([])
		self.kr = matrix([])


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

	# TODO: what if one or more vehicles lost connection?
	def neighborsPosesUpdate(self, data, carNum):
		pose_Coord = [data.position.x, data.position.y, data.orientation.z]
		self.neighborPosesDict[carNum] = pose_Coord

	def neighborsFrontDistUpdate(self, data, carNum):
		self.neighborFrontDistDict[carNum] = data.data
		# print(self.neighborFrontDistDict)

	def agentPoseUpdate(self, data):
		self.agent_pos = [data.poses[-1].pose.position.x, data.poses[-1].pose.position.y]
		self.agent_angle = data.poses[-1].pose.orientation.z

	# Update instant velocity as feedback.
	def feedbackVelUpdate(self, data):
		self.agent_x_vel_feedback = data.linear.x

	# Update if vehicle is moving or stop based on God switch message
	def switchUpdate(self, data):
		self.godSwitch = data.data

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
		# print('steering angle: ' + str(self.agent_z_vel))

 	# **********************************************************************

	def potentialField(self):
		agent_x_pos = self.agent_pos[0]
		agent_y_pos = self.agent_pos[1]
		detect_x_range = [-5,70]
		detect_y_range = [-16,16]
		boundaries = [-8.0, 8.0]
		attr_dpdx = 0
		attr_dpdy = 0
		repl_dpdx = 0
		repl_dpdy = 0

		covX = 45
		covY = 8
		covBound_left = 0.7 * exp(abs(self.agent_angle*160/(boundaries[0]-agent_y_pos)))
		covBound_right = 0.7 * exp(abs(self.agent_angle*160/(boundaries[1]-agent_y_pos)))
		boundGain_gaussian_left = exp(-self.agent_angle*40/abs(boundaries[0]-agent_y_pos))
		boundGain_gaussian_right = exp(self.agent_angle*40/abs(boundaries[1]-agent_y_pos))
		# print(boundGain_gaussian_left, boundGain_gaussian_right)

		obstacleGain = 70
		heuristicGain = 1


		neighborPosesList = list(self.neighborPosesDict.values())

		goalPosX, goalPosY = self.getGoal(20, 12, 13)	# (distDiff, frontFistThreshold, goalOffset)
		# print('the goal is: ' + str([goalPosX, goalPosY]))

		if len(neighborPosesList) == 0:
			print("No obstacle detected.")
		else:
			for p in neighborPosesList:
				obsX = p[0]
				obsY = p[1]
				if detect_x_range[0]<=(p[0]-agent_x_pos)<=detect_x_range[1] and detect_y_range[0]<=(p[1]-agent_y_pos)<=detect_y_range[1]:
					# repulsive gradient vectors. the oroginal gaussian function is: 
					P = obstacleGain * exp(-0.5*(-obsY + agent_y_pos)**2/covY - 0.5*(-obsX + agent_x_pos)**2/covX)/(2*pi*sqrt(covX*covY))
					# dirX = 0.25*(-2*obsX + 2*agent_x_pos)*exp(-0.5*(-obsY + agent_y_pos)**2/covY - 0.5*(-obsX + agent_x_pos)**2/covX)/(pi*covX*sqrt(covX*covY))
					dirX = 0
					dirY = 0.25*(-2*obsY + 2*agent_y_pos)*exp(-0.5*(-obsY + agent_y_pos)**2/covY - 0.5*(-obsX + agent_x_pos)**2/covX)/(pi*covY*sqrt(covX*covY))
					repl_dpdx = repl_dpdx + P * dirX / sqrt(dirX**2 + dirY**2)
					repl_dpdy = repl_dpdy + P * dirY / sqrt(dirX**2 + dirY**2)
		
		if boundaries[0]>=agent_y_pos+detect_y_range[0]: 	# if left bound is inside of potential field range
			repl_dpdy = repl_dpdy + boundGain_gaussian_left*(exp(-0.5*(agent_y_pos-boundaries[0])/covBound_left*(agent_y_pos-boundaries[0]))/sqrt(covBound_left*2*pi))			# gaussian
		if boundaries[1]<=agent_y_pos+detect_y_range[1]:	# if right bound is inside of potential field range
			repl_dpdy = repl_dpdy + -1*boundGain_gaussian_right*(exp(-0.5*(agent_y_pos-boundaries[1])/covBound_right*(agent_y_pos-boundaries[1]))/sqrt(covBound_right*2*pi))		# gaussian
		# print("walls in y gradient is: " + str(float((repl_dpdy - P * dirY / sqrt(dirX**2 + dirY**2)).subs({x:agent_x_pos, y:agent_y_pos}))))
		# print("walls in y gradient is: " + str(float(repl_dpdy.subs({x:agent_x_pos, y:agent_y_pos}))))
		
		attr_dpdx = heuristicGain

		if 0 < goalPosX - agent_x_pos < 12:
			attr_dpdy = 0.2*sign(goalPosY - agent_y_pos)
		else:
			attr_dpdy = 0

		dx = attr_dpdx + repl_dpdx
		dy = attr_dpdy + repl_dpdy

		self.gradientLength = sqrt(dy**2 + dx**2)
		self.gradientAngle = atan2(dy, dx)
		# print(dx, dy, self.vehicleState)
		# print(self.gradientAngle*57.3, self.vehicleState)

	def getGoal(self, distDiff, frontFistThreshold, goalOffset):
		agent_x_pos = self.agent_pos[0]
		agent_y_pos = self.agent_pos[1]

		# check if there has neighbor vehicles first. If not, arbitrarily set a goal at agent's front.
		if len(self.neighborPosesDict) == 0:
			print("I can't see cars on the road.")
			return agent_x_pos + 1, agent_y_pos

		# transform position information from dictionary format {veh_num:[xPos, yPos], ... } 
		# to list format: [ [veh_num, xPos, yPos], ... ] for easier data processing
		neighborInfoList = []
		for key, value in self.neighborPosesDict.iteritems():
			vehiclePoseInfo = [key] + value
			neighborInfoList.append(vehiclePoseInfo)

		# sort the info list based on each element's x position
		neighborInfoList = sorted(neighborInfoList, key=lambda x:x[1])
		
		for posInfo in neighborInfoList:
			neighbor_num = posInfo[0]
			neighbor_x = posInfo[1]
			neighbor_y = posInfo[2]
			# Check if the neighbor vehicle is is the closed vehicles. If yes, jump to check next neighbor vehicle.
			if distDiff > agent_x_pos - neighbor_x:
				# print(agent_x_pos - neighbor_x)
				# Check if the neighbor has its corresponding laser information. If not, report error.
				if self.neighborFrontDistDict.has_key(neighbor_num):

					# Check if the chosen neighbor vehicle has enough distance to fit agent vehicle:
					if self.neighborFrontDistDict[neighbor_num] >= frontFistThreshold:
						print("I'm chasing the car "+ str(neighbor_num) + " where it's in " + str([neighbor_x, neighbor_y]))
						return neighbor_x + goalOffset, neighbor_y
					else:
						pass	# if the neighbor vehicle doesn't have enough room to fit, jump to check next neighbor vehicle.
				else:
					rospy.logerr('vehicle ' + str(neighbor_num) + ' laser data not available')
					return
			else:
				pass

		# if the neighbor vehicle is in front of all neighbor vehicles, arbitrarily set a goal at agent's front.
		print("I am ahead of all the vehicles.")
		return agent_x_pos + 1, agent_y_pos
		


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

	def speedController(self, low_speed, high_speed, threshDist):
		if self.frontDist >= threshDist and -pi/8<self.agent_angle<pi/8:
			self.agent_desire_x_vel = high_speed
			self.vehicleState = 2
		elif self.frontDist >= 5:
			self.agent_desire_x_vel = low_speed
			self.vehicleState = 5
		else:
			self.agent_desire_x_vel = 0
			self.agent_x_vel = 1
			self.vehicleState = 1

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
		PD_ctrl = 1 * self.steeringError + 0 * (self.steeringError - self.steeringError_last)/(time.time() - self.lastMsg)
		if abs(PD_ctrl) <= limit:
			self.agent_z_vel = PD_ctrl
		else:
			self.agent_z_vel = sign(PD_ctrl) * limit

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
			for n in range(1, len(self.neighborPosesDict)+1):
				xPos = self.neighborPosesDict[n][0]
				yPos = self.neighborPosesDict[n][1]
				zAngle = self.neighborPosesDict[n][2]
				dataInRow.append(xPos)
				dataInRow.append(yPos)
				dataInRow.append(zAngle)
			filewriter.writerow(dataInRow)


def main(argv):
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
		if node.godSwitch:
			node.potentialField()
			node.steeringCtrl(12, 0.13, 9, 0.2)	# (high_speed, highLimit, low_speed, lowLimit)
			node.speedController(9, 12, 15) 		# (low-speed, high-speed, threshold of front distance)
			node.P_Controller(3, 0.1)			# (acceleration, tolerance)
			node.speedPublisher()
			node.writeData()
		else:
			print('Waiting for start...')
			rospy.sleep(1)
		node.recordLastMsg()
		rate.sleep()



if __name__ == '__main__':
	try:
		main(sys.argv)
	except rospy.ROSInterruptException:
		pass