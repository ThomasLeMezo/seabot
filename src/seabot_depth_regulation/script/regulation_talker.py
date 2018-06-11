#!/usr/bin/env python

import rospy
from seabot_depth_regulation.msg import *
from seabot_fusion.msg import *
from std_srvs.srv import *

def set_flash(enable):
    try:
        resp = flash_enable(enable)
    except rospy.ServiceException, e:
        rospy.logwarn("[Regulation_Talker] Fail to call Flash Enable");

def set_zero_depth():
    try:
        resp = fusion_zero_depth()
        rospy.loginfo("[Regulation_Talker] Zero ")
    except rospy.ServiceException, e:
        rospy.logwarn("[Regulation_Talker] Fail to call Flash Enable");

def set_depth(val, duration=0):
  msg = DepthPose()
  msg.depth = val
  set_flash(True)
  piston_position.publish(msg)
  rospy.sleep(10)
  set_flash(False)
  if(duration-10>0):
    rospy.sleep(duration-10)

def talker():
  global flash_enable, fusion_zero_depth, piston_position
  rospy.init_node('regulation_talker_node', anonymous=True)

  piston_position = rospy.Publisher('/regulation/depth_set_point', DepthPose, queue_size=1)
  rospy.wait_for_service('/fusion/zero_depth')
  rospy.wait_for_service('/driver/power/flash_led')
  fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Empty)
  flash_enable = rospy.ServiceProxy('/driver/power/flash_led', SetBool)

  #Wait 30s before init
  rospy.sleep(30.0)

  set_zero_depth()
  set_depth(0.5, 10*60)
  set_depth(1.2, 10*60)
  set_depth(0.0)

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
