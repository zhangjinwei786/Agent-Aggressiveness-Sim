#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool



def run(state):
	rospy.init_node('Main_Control', anonymous=True)
	pub_godSwitch = rospy.Publisher('godSwitch_topic', Bool, queue_size = 2)
	print('press "s" to start and stop simulation')
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
				print("Simulation Stops.")


if __name__ == '__main__':
    try:
        run(0)
    except rospy.ROSInterruptException:
	pass