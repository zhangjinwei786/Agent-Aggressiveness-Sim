#!/usr/bin/env python
import rospy
import math
import time
from numpy import matrix
from numpy.linalg import inv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

global setupFlag
setupFlag = True

global stopFlag
stopFlag = True

def callback(data):
	global setupFlag
	global last_y_pos
	global last_check_time
	desired_y_pos = -2
	if setupFlag:
		steeringCtrlConfig(1, 1, 2)
		last_y_pos = desired_y_pos
		last_check_time = time.time()
		setupFlag = False

	current_time = time.time()
	current_y_pos = data.poses[-1].pose.position.y
	steeringAngle = steeringCtrl(desired_y_pos, current_y_pos, last_y_pos, (current_time - last_check_time))
	speedPublisher(3,steeringAngle)

	last_y_pos = current_y_pos
	last_check_time = current_time

def speedPublisher(linearSpeed, angularSpeed):
	global pub_speed
	newVel = Twist()
	newVel.linear.x = linearSpeed
	newVel.angular.z = angularSpeed
	pub_speed.publish(newVel)

def steeringCtrlConfig(gamma, omega0, zeta):
	# gamma = 0.5
	# omega0 = 1
	# zeta = 0.7
	global A
	global B
	global K
	global kr
	A = matrix([[0,1],[0,0]])
	B = matrix([[gamma],[1]])
	C = matrix([1,0])
	k1 = math.pow(omega0,2)
	k2 = 2*zeta*omega0-0.5*k1
	K = matrix([k1, k2])
	krMatrix = -1 / (C*inv(A-B*K)*B)
	kr = krMatrix[0,0]

def steeringCtrl(desired_y_pos, current_y_pos, last_y_pos,timeInterval):
	global A
	global B
	global K
	global kr
	x = matrix([[current_y_pos],[((current_y_pos - last_y_pos)/timeInterval)]])
	u = -K*x + kr*desired_y_pos
	xdot = A*x+B*u
	return xdot[0,0]

def stopCmd():
	global pub_speed
	global stopFlag
	if stopFlag:
		speedPublisher(0,0)
		stopFlag = False


def run():
	global pub_speed
	rospy.init_node('CarConfig', anonymous=True)
	rospy.Subscriber("/one_front_car/path", Path, callback)
	pub_speed = rospy.Publisher('/one_front_car/cmd_vel',Twist, queue_size = 3)

	# while not rospy.is_shutdown():
	rospy.on_shutdown(stopCmd)
	rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
	pass