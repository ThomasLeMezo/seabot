#!/usr/bin/env python

from math import *
import rospy
from seabot_piston_driver.msg import *

def talker():
  rospy.init_node('sinus_node', anonymous=True)
  piston_position = rospy.Publisher('/driver/piston/position', PistonPosition, queue_size=1)

  f = 1.0/60.0

  while not rospy.is_shutdown():
    cmd = 100.0*sin(rospy.get_time()*2.0*pi*f) + 1000
    piston_position.publish(cmd)
    
    rospy.sleep(1)

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
