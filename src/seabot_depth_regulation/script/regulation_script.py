#!/usr/bin/env python

import rospy
from seabot_depth_regulation.msg import *
from seabot_fusion.msg import *
from std_srvs.srv import *
from seabot_power_driver.msg import Battery
from seabot_mission.msg import *

battery_warning = False

def set_flash(enable):
  global flash_enable
  try:
      resp = flash_enable(enable)
  except rospy.ServiceException, e:
      rospy.logwarn("[Regulation_Talker] Fail to call Flash Enable");

def callback_battery(data):
    global battery_warning
    if(data.battery1 < 10.0 and data.battery2 < 10.0 and data.battery3<10.0 and data.battery4 < 10.0):
      battery_warning = True

#def set_zero_depth():
#  global fusion_zero_depth
#  try:
#      resp = fusion_zero_depth()
#      rospy.loginfo("[Regulation_Talker] Zero ")
#  except rospy.ServiceException, e:
#      rospy.logwarn("[Regulation_Talker] Fail to call Zero Depth");

def set_depth(val, duration=0):
  global piston_position
  msg = Waypoint()
  msg.depth = val
  set_flash(True)
  piston_position.publish(msg)
  rospy.sleep(10)
  set_flash(False)
  if(duration-10>0):
    rospy.sleep(duration-10)

def talker():
  global flash_enable, fusion_zero_depth, piston_position, battery_warning
  rospy.init_node('regulation_talker_node', anonymous=True)

  piston_position = rospy.Publisher('/regulation/depth_set_point', Waypoint, queue_size=1)
#  rospy.wait_for_service('/fusion/zero_depth')
  rospy.wait_for_service('/driver/power/flash')
  rospy.wait_for_service('/driver/power/sleep_mode')
#  fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Trigger)
  flash_enable = rospy.ServiceProxy('/driver/power/flash', SetBool)
  sleep_enable = rospy.ServiceProxy('/driver/power/sleep_mode', Empty)

  #Wait 6s before init
  rospy.loginfo("[Regulation_Talker] Wait 6 s")
  rospy.sleep(6.0)
#  set_zero_depth()
#  rospy.sleep(3.0)

  for d in range(1, 17, 2):
    rospy.loginfo("[Regulation_Talker] Depth %i", d)
    set_depth(d, 60*60)
  for d in range(16, 0, -2):
    rospy.loginfo("[Regulation_Talker] Depth %i", d)
    set_depth(d, 60*60)

  for i in range(3):
    rospy.loginfo("[Regulation_Talker] Depth 5")
    set_depth(5, 60*60)
    rospy.loginfo("[Regulation_Talker] Depth 15")
    set_depth(15, 60*60)
    rospy.loginfo("[Regulation_Talker] Depth 0")
    set_depth(0, 60*60)

    if battery_warning:
      set_depth(0.0)
      break
  set_depth(0.0)

  rospy.sleep(300)
  for i in range(2):
   try:
       resp = sleep_enable()
       rospy.loginfo("[Regulation_Talker] Sleep mode activated ")
   except rospy.ServiceException, e:
       rospy.logwarn("[Regulation_Talker] Fail to sleep mode");
   rospy.sleep(1.0)

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
