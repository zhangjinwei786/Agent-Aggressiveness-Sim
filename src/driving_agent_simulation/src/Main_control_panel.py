#!/usr/bin/env python

# Author: Jinwei Zhang

# This node is a simple panel to control the start and pause of the simulation.
# It has one publisher, and publish boolean information to a topic called
# 'godSwitch_topic'.

import rospy
from std_msgs.msg import Bool

# This function set a simple FSM. When the state changs from 0 to 1, it publish
# start signal; when it change from 1 to 0, it publish pause signal.
def run(state):
	rospy.init_node('Main_Control', anonymous=True)
	pub_godSwitch = rospy.Publisher('godSwitch_topic', Bool, queue_size = 2)
	print('press "s" to start and pause simulation')
	while not rospy.is_shutdown():
		rawIn = raw_input('')
		if state == 0:
			if rawIn == 's':
				pub_godSwitch.publish(True)
				state = 1
				print("Simulation Starts.")
		elif state == 1:
			if rawIn == 's':
				pub_godSwitch.publish(False)
				state = 0
				print("Simulation Pause.")


if __name__ == '__main__':
    try:
	# starting at stop (or pause) state.
        run(0)
    except rospy.ROSInterruptException:
	pass
