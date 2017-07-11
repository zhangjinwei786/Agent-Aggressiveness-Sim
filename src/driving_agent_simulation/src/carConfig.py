#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def run():
	global pub_Speed

	rospy.init_node('CarConfig', anonymous=True)
	pub_speed = rospy.Publisher('/catvehicle/cmd_vel',Twist, queue_size = 3)
	while True
	#if rospy.is_shutdown():
	rospy.on_shutdown(emergencyStop)
	rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
	pass
