#!/usr/bin/env python

import rospy
import sys, time
from numpy import matrix, sign
from numpy.linalg import inv
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32
from math import sqrt, atan2, pi
from sympy import *


class agent_driver:

	def __init__(self, ns, agent_x_pos, agent_y_pos, frontVehiclesNum):
		self.ns = ns
		rospy.init_node('agent_driver', anonymous=True)

		# Setup publishers and subscribers
		# the format(ns) looks for the namespace in the ros parameter server
		rospy.Subscriber('/godSwitch_topic', Bool, self.statusUpdate)
		rospy.Subscriber('front_dist'.format(ns), Float32, self.distUpdate)
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

		# gradient vector as polar coordinate ststem. Initialize the magnitude and angle as 0.
		self.gradientLength = 0
		self.gradientAngle = 0

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

		# TODO: set two states for the vehicle: 
		#		0 -> slow-speed state
		#		1 -> normal-speed state
		#		Starting at slow speed state.
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

	# *********** subscriber callbacks and publish functions ***************

	# Two callback functions to update car's instant position, neighbor cars'
	# instant positions

	# TODO: what if one or more vehicles lost connection?
	def neighborsPosesUpdate(self, data, carNum):
		pose_Coord = [data.position.x, data.position.y]
		self.neighborsPosesDict[carNum] = pose_Coord

	def agentPoseUpdate(self, data):
		self.agent_pos = [data.poses[-1].pose.position.x, data.poses[-1].pose.position.y]
		self.agent_angle = data.poses[-1].pose.orientation.z

	def statusUpdate(self, data):
		self.godSwitch = data.data

	def distUpdate(self, data):
		pass

	# Publish car's movement (linear and angular velocities to Gazebo)
	def speedPublisher(self):
		newVel = Twist()
		newVel.linear.x = self.agent_x_vel
		newVel.angular.z = self.agent_z_vel
		self.pub_speed.publish(newVel)
		# print('agent speed: ' + str(self.agent_x_vel))
		# print('steering angle: ' + str(float(self.agent_z_vel*180/pi)))

 	# **********************************************************************

	def potentialField(self):
		x = Symbol('x')
		y = Symbol('y')
		agent_x_pos = self.agent_pos[0]
		agent_y_pos = self.agent_pos[1]
		detect_x_range = [-20,100]
		detect_y_range = [-16,16]
		boundaries = [-7.5, 7.5]
		attrCloud = 0
		replCloud = 0
		covX = 0.001
		covY = 0.006
		covBound = 10
		heuristicGain = self.agent_x_vel*1.5
		neighborPosesList = list(self.neighborsPosesDict.values())

		# Temporarily set goal point as frontier position
		goalPosX = agent_x_pos + detect_x_range[1]
		goalPosY = -2

		if len(neighborPosesList) == 0:
			print("No obstacle detected.")
		else:
			for p in neighborPosesList:
				obsX = p[0]
				obsY = p[1]
				if detect_x_range[0]<=(obsX-agent_x_pos)<=detect_x_range[1] and detect_y_range[0]<=(obsY-agent_y_pos)<=detect_y_range[1]:
					replCloud = replCloud + 1/( (sqrt((x-obsX)**2)*covX) + (sqrt((y-obsY)**2)*covY) )
		
		if boundaries[0]>=agent_y_pos+detect_y_range[0]:
			replCloud = replCloud + covBound/(sqrt((boundaries[0]-y)**2))
		if boundaries[1]<=agent_y_pos+detect_y_range[1]:
			replCloud = replCloud + covBound/(sqrt((boundaries[1]-y)**2))
		# attrCloud = sqrt((goalPosX - x)**2 + (goalPosY - y)**2) * heuristicGain
		attrCloud = (x - goalPosX)* heuristicGain
		potCloud = replCloud - attrCloud

		dpdx = diff(potCloud, x)
		dpdy = diff(potCloud, y)
		dx = float(dpdx.subs({x:agent_x_pos, y:agent_y_pos}))
		dy = float(dpdy.subs({x:agent_x_pos, y:agent_y_pos}))
		self.gradientLength = sqrt(dy**2 + dx**2)
		self.gradientAngle = atan2(-dy,-dx)
		print(self.gradientLength, float(self.gradientAngle/pi*180))


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
		if self.vehicleState == 0: # at stable state..
			self.agent_desire_x_vel = 0
			if self.gradientLength >= gradLenThresh: # from 0 to 1
				self.vehicleState = 1
				print("agent: low speed")
			elif self.gradientLength < gradLenThresh and -pi/2<self.gradientAngle<pi/2: # from 0 to 2
				self.vehicleState = 2
				print("agent: normal speed")
		elif self.vehicleState == 1: # at low-speed state..
			self.agent_desire_x_vel = low_speed
			if self.gradientLength < gradLenThresh and (-pi<=self.gradientAngle<=-pi/2 or pi/2<=self.gradientAngle<=pi): # from 1 to 0
				self.vehicleState = 0
				print("agent: stop")
			elif self.gradientLength < gradLenThresh and -pi/2<self.gradientAngle<pi/2: # from 1 to 2
				self.vehicleState = 2
				print("agent: normal speed")
		elif self.vehicleState == 2: # at high-speed state..
			self.agent_desire_x_vel = high_speed
			if self.gradientLength < gradLenThresh and (-pi<=self.gradientAngle<=-pi/2 or pi/2<=self.gradientAngle<=pi): # from 2 to 0
				self.vehicleState = 0
				print("agent: stop")
			elif self.gradientLength >= gradLenThresh: # from 2 to 1
				self.vehicleState = 1
				print("agent: low speed")
		self.agent_x_vel = self.agent_desire_x_vel
		# print(self.gradientLength)

	'''
	# P controller: to avoid accelerating too fast and drifting.
	# Known current speed and desire speed, the car can accelerate/decelerate to reach
	# the speed in a certain value.
	# Small error can also be smoothed by setting tolerance value.
	def P_Controller(self, accel, tolerance):
		accel = abs(accel)
		error = self.agent_desire_x_vel - self.agent_x_vel
		timeStamp = time.time() - self.lastMsg
		if abs(error) > tolerance:
			self.agent_x_vel = self.agent_x_vel + sign(error) * accel * timeStamp
		else:
			self.agent_x_vel = self.agent_desire_x_vel
	'''

	# Setup car's steering dynamic and PD+feed forward controller
	def steeringCtrlConfig(self):
		self.A = matrix([[0,1],[0,0]])
		self.B = matrix([[self.gamma],[1]])
		self.C = matrix([1,0])
		k1 = pow(self.omega0,2)
		k2 = 2*self.zeta*self.omega0-0.5*k1
		self.K = matrix([k1, k2])
		krMatrix = -1 / (self.C*inv(self.A-self.B*self.K)*self.B)
		self.kr = krMatrix[0,0]

	# steering controller
	def steeringCtrl(self, highLimit, lowLimit):
		error = self.gradientAngle - self.agent_angle
		if self.vehicleState == 1:
			limit = lowLimit
		elif self.vehicleState == 2:
			limit = highLimit
		else:
			limit = 0.1
		if abs(error) <= limit:
			self.agent_z_vel = error*0.8
		else:
			self.agent_z_vel = sign(error)* limit

	# Update time stamp for every loop.
	def recordLastMsg(self):
		self.lastMsg = time.time()

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
			node.steeringCtrlConfig()
			node.startFlag = False
		if node.godSwitch:
			node.potentialField()
			node.steeringCtrl(0.2, 0.35)		# (Limit in high speed, Limit in low speed)
			node.speedController(3, 9, 8) 	# (low-speed, high-speed, threshold of gradient magnitude)
			# node.P_Controller(1, 0.1) 		# (acceleration, tolerance)
			node.speedPublisher()
		else:
			print('Waiting for start...')
			rospy.sleep(1)
		rate.sleep()



if __name__ == '__main__':
    try:
    	main(sys.argv)
        # listener('catvehicle')
    except rospy.ROSInterruptException:
        pass