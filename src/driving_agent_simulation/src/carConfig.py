#!/usr/bin/env python
import rospy
import math
from numpy.linalg import inv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

def callback(data):
	global last_y_pos
	global last_check_time
	current_y_pos = data.poses.pose.y
	global pub_speed
	newVel = Twist()
	newVel.linear.x = 3
	pub_speed.publish(newVel)

def sterringCtrlConfig(gamma, omega0, zeta):
	# gamma = 0.5
	# omega0 = 1
	# zeta = 0.7
	global A
	global B
	global K
	global kr
	A = [[0,1],[0,0]]
	B = [[gamma],[1]]
	k1 = math.pow(omega0,2)
	k2 = 2*zeta*omega0-0.5*k1
	K = [k1, k2]
	kr = -1 / (C*inv(A-B*K)*B)

def SterringCtrl(desired_y_pos, current_y_pos, last_y_pos,timestamp):
	global A
	global B
	global K
	global kr
	x = [[current_y_pos],[(current_y_pos - last_y_pos)/timestamp]]
	u = -K*x + kr*[[desired_y_pos],[0]]
	xdot = Ax+Bu


def run():
	global pub_speed

	rospy.init_node('CarConfig', anonymous=True)
	rospy.Subscriber("/one_front_car/path", Path, callback)
	pub_speed = rospy.Publisher('/one_front_car/cmd_vel_safe',Twist, queue_size = 3)

	while not rospy.is_shutdown():
		# rospy.on_shutdown(emergencyStop)
		rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
	pass
