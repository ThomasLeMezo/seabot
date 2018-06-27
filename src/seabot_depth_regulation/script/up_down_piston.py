#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from seabot_piston_driver.srv import *
from seabot_piston_driver.msg import *

def talker():
  rospy.init_node('up_down_piston_node', anonymous=True)

  piston_position = rospy.Publisher('/driver/piston/position', PistonPosition, queue_size=1)

  delta_time = rospy.get_param('~delta_time', 180.0)
  sleep_time = rospy.Duration(delta_time)

  down = True
  while not rospy.is_shutdown():
    if(down == True):
      piston_position.publish(0)
      down=False
    else:
      piston_position.publish(1200)
      down=True

    rospy.sleep(sleep_time)

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
