#!/usr/bin/env python

from math import *
import rospy
from seabot_piston_driver.msg import *

def talker():
  rospy.init_node('test_piston', anonymous=True)

  piston_position = rospy.Publisher('/position', PistonPosition, queue_size=1)
  rospy.loginfo("Commande de position")
  delta_time = rospy.get_param('~delta_time', 180.0)
  sleep_time = rospy.Duration(delta_time)

  while not rospy.is_shutdown():
	rospy.sleep(2)
	piston_position.publish(355*4)
    	#355*4
	#1070 + 355*4
	#1050 + 368*4 25 pourcent verin + gros piston rentres
	#rospy.sleep(100)
	#piston_position.publish(5700)
	#1411.5*4 max correspond a 5650
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
