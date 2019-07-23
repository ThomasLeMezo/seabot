#!/usr/bin/env python

from math import *
import rospy
from seabot_piston_driver.msg import *
import time

def talker():
  rospy.init_node('test_piston_dive_and_surface', anonymous=True)

  piston_position = rospy.Publisher('/position', PistonPosition, queue_size=1)
  delta_time = rospy.get_param('~delta_time', 180.0)
  sleep_time = rospy.Duration(delta_time)

#  while not rospy.is_shutdown():
  for i in range(2):
	rospy.sleep(10)

	rospy.loginfo("Dive")
	piston_position.publish(5700)

	rospy.sleep(200)

	rospy.loginfo("Large piston out")
	piston_position.publish(4*368)

	rospy.sleep(200)

	rospy.loginfo("Surface")
	piston_position.publish(0)
	rospy.sleep(190)


	print("i (0;2) = ", i)
  print("the test is finished")


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
