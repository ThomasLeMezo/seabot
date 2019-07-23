#!/usr/bin/env python

from math import *
import rospy
from seabot_piston_driver.msg import *

def callback(data):
	global flag
	if data.switch_halfway == True and flag == False:
		print("switch_halfway_position_on = ", data.position)
		flag = True

	elif data.switch_halfway == False and flag == True:
		print("switch_halfway_position_off = ", data.position)
		flag = False


	print("piston_position = ", data.position)

def listener_switch_halfway():
	rospy.init_node('listener_switch_halfway', anonymous = True)
	rospy.Subscriber("/driver/piston/state", PistonState, callback)
	rospy.spin()

if __name__ == '__main__':
  global flag
  flag = False
  try:
    listener_switch_halfway()
  except rospy.ROSInterruptException:
    pass
