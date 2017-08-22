#!/usr/bin/env python

import rospy
import sys, getopt, math, time, random
from numpy import matrix, sign
from numpy.linalg import inv
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool


class npc_driver:

	def __init__(self,ns,x_pos,y_pos):
		self.ns = ns
		rospy.init_node('npc_driver', anonymous=True)

		# Setup publishers and subscribers
		# the format(ns) looks for the namespace in the ros parameter server
		rospy.Subscriber('path'.format(ns), Path, self.poseUpdate)
		rospy.Subscriber('front_dist'.format(ns), Float32, self.distUpdate)
		rospy.Subscriber('/godSwitch_topic', Bool, self.switchUpdate)
		rospy.Subscriber('vel'.format(ns), Twist, self.feedbackVelUpdate)
		self.pub_speed = rospy.Publisher('cmd_vel'.format(ns), Twist, queue_size = 2)
		self.pub_pose = rospy.Publisher('pose'.format(ns), Pose, queue_size = 2)

		# TODO: God switch is the main control switch of whole simluation. The movement
		#		doesn't start until it turns on. Set switch off in default.
		self.godSwitch = False

		# initial position is based on parameters.
		self.x_pos = x_pos
		self.y_pos = y_pos

		# initial last position: same as current position
		self.x_last_pos = x_pos
		self.y_last_pos = y_pos

		# initial linear and angular velocity as 0.
		self.x_vel = 0
		self.z_vel = 0

		# initial desired speed as 0.
		self.desire_x_vel = 0
		
		# current speed as feedback. Initialized as zero.
		self.x_vel_feedback = 0

		# initial front distance as 10 meters
		self.frontDist = 10

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

	# ************* subscriber callbacks and publish functions *************
	# TODO: Three callback functions to update car's instant status: current position,
	#		front distance and God switch's status.

	# Update vehicle's position and publish the message to agent vehicle
	def poseUpdate(self, data):
		self.x_pos = data.poses[-1].pose.position.x
		self.y_pos = data.poses[-1].pose.position.y
		self.pub_pose.publish(data.poses[-1].pose)

	# Update vehicle's front distance
	def distUpdate(self, data):
		self.frontDist = data.data

	# Update if vehicle is moving or stop based on God switch message
	def switchUpdate(self, data):
		self.godSwitch = data.data

	# Update instant velocity as feedback.
	def feedbackVelUpdate(self, data):
		self.x_vel_feedback = data.linear.x

	# Publish car's movement (linear and angular velocities to Gazebo)
	def speedPublisher(self):
		newVel = Twist()
		newVel.linear.x = self.x_vel
		newVel.angular.z = 0 # self.z_vel
		self.pub_speed.publish(newVel)


	# ************* linear speed controller helper functions *************


	# Use front distance to decide which state should be switched. if distance is
	# larger than a certain range, the vehicle switches to normal-speed state; 
	# otherwise switches to low-speed state.
	# == == == == == == == == == == ==
	# 1. slow-speed state (as 0): vehicle runing in the lowest speed in the speed range.
	# 2. nornal-speed state (as 1): vehicle running in the speed  that rundomly generated 
	# 	 in the valid speed range.

	def speedController(self, criticalDist, minSpeed, maxSpeed):
		if self.vehicleState == 0:
			if self.frontDist > criticalDist:	# switch state from low-speed to normal-speed
				self.vehicleState = 1
				self.desire_x_vel = random.randint(minSpeed,maxSpeed)
				print(self.ns + ' switch to normal-speed mode: ' + str(self.desire_x_vel) + ' m/s')
		elif self.vehicleState == 1:
			if self.frontDist <= criticalDist:	# switch state from normal-speed to low-speed
				self.vehicleState = 0
				self.desire_x_vel = minSpeed
				print(self.ns + ' switch to low-speed mode: ' + str(minSpeed) + ' m/s')

	# P controller: to avoid accelerating too fast and drifting.
	# Known current speed and desire speed, the car can accelerate/decelerate to reach
	# the speed in a certain value.
	# Small error can also be smoothed by setting tolerance value.
	def P_Controller(self, accel, tolerance):
		accel = abs(accel)
		error = self.desire_x_vel - self.x_vel_feedback
		timeStamp = time.time() - self.lastMsg
		if abs(error) > tolerance:
			self.x_vel = self.x_vel_feedback + sign(error) * accel * timeStamp
		else:
			self.x_vel = self.desire_x_vel


	# ************* Steering controller helper functions *************

	# Setup car's steering dynamic feedback controller.
	def steeringCtrlConfig(self):
		self.A = matrix([[0,1],[0,0]])
		self.B = matrix([[self.gamma],[1]])
		self.C = matrix([1,0])
		k1 = math.pow(self.omega0,2)
		k2 = 2*self.zeta*self.omega0-0.5*k1
		self.K = matrix([k1, k2])
		krMatrix = -1 / (self.C*inv(self.A-self.B*self.K)*self.B)
		self.kr = krMatrix[0,0]

	# steering controller
	def steeringCtrl(self, desired_y_pos, limit):
		y_vel = (self.y_pos - self.y_last_pos)/(time.time() - self.lastMsg)
		x = matrix([[self.y_pos],[y_vel]])
		u = -1*self.K*x + self.kr*desired_y_pos
		xdot = self.A*x+self.B*u

		self.y_last_pos = self.y_pos
		self.x_last_pos = self.x_pos
		if abs(xdot[0,0]) <= limit:
			self.z_vel = xdot[0,0]
		else:
			self.z_vel = sign(xdot[0,0])* limit


	# Update time stamp for every loop.
	def recordLastMsg(self):
		self.lastMsg = time.time()


def main(argv):

    # we eventually get the ns (namespace) from the ROS parameter server for this node
	ns = sys.argv[1]
	init_pos_x = float(sys.argv[2])
	init_pos_y = float(sys.argv[3])
	rospy.loginfo('NPC Driver: ' + ns + ' successfully set.')
	rospy.loginfo('Initial position set in random: ' + '[' + str(init_pos_x) + ',' + str(init_pos_y) + ']')
	node = npc_driver(ns,init_pos_x, init_pos_y)
	rate = rospy.Rate(20) # run at 20Hz
	
	while not rospy.is_shutdown():
		if node.startFlag:
			node.steeringCtrlConfig()
	    	node.startFlag = False
		if node.godSwitch:
			node.speedController(10, 5, 7)			# (critical distance, minSpeed, maxSpeed)
			node.P_Controller(3, 0.1)				# (acceleration, tolerance)

			## Since in this simulation, front vehicles are only go and keep straight line. So
			## those steering controller are noe necessary to be used right now.

			# node.steeringCtrl(init_pos_y, 0.3)		# (reference input in y axis, steering_limit)
			node.speedPublisher()
		else:
			# reset vehicleState as slow-speed mode, and set desire speed as zero.
			# in this way the vehicle will slow down and stop instead of sudden stop
			# and drift.
			node.vehicleState = 0
			node.desire_x_vel = 0
			node.P_Controller(3, 0.05)
			node.speedPublisher()
		node.recordLastMsg()
		rate.sleep()

if __name__ == '__main__':
    try:
    	main(sys.argv)
        # listener('catvehicle')
    except rospy.ROSInterruptException:
        pass