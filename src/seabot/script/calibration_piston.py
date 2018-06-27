#!/usr/bin/env python

import rospy

from seabot_piston_driver.msg import PistonState
from std_msgs.msg import Float64MultiArray
from seabot_fusion.msg import DepthPose
from seabot_depth_regulation.msg import RegulationDebug

from seabot_piston_driver.srv import PistonPosition
from std_srvs.srv import *
from seabot_depth_regulation.srv import *

import numpy as np
from math import *

depth = 0

### Piston
start_piston_position = 700
piston_position = 0

## ToDo : service reset zero pression + depth_set_point

def callback_fusionu_depth(data):
    global depth, velocity
    depth = data.depth
    velocity = data.velocity

def set_piston_position(target):
    try:
        resp = piston_position_set_point(target)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_piston] Fail to call Piston Set point");

def set_flash(enable):
    try:
        resp = flash_enable(enable)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_piston] Fail to call Flash Enable");

def set_zero_depth():
    try:
        resp = fusion_zero_depth()
        rospy.loginfo("[Calibration_piston] Zero ")
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_piston] Fail to call Flash Enable");


def regulation_node():
    global flash_enable, fusion_zero_depth, start_piston
    global depth, velocity, piston_position, piston_position_set_point
    rospy.init_node('calibration_piston_node', anonymous=True)

    # sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusionu_depth ,queue_size=1)
    
    rospy.loginfo("[Calibration] Wait for Services")
    rospy.wait_for_service('/fusion/zero_depth')
    rospy.wait_for_service('/driver/power/flash_led')

    fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Trigger)
    flash_enable = rospy.ServiceProxy('/driver/power/flash_led', SetBool)

    time_start = rospy.Time().now()

    ######################################################################
    ########## Main
    ######################################################################
    
    rospy.loginfo("[Calibration_piston] Wait 6 s")
    rospy.sleep(3.0)
    set_zero_depth()
    set_flash(True)
    rospy.sleep(3.0)
    set_flash(False)
    set_piston_position(start_piston_position)
    rospy.sleep(start_piston_position/7.0)

    for i in range(3):
        rospy.loginfo("[Calibration] Start Calibration %i", i)
        set_piston_position(start_piston_position)
        rospy.sleep(10.0)
        piston_position = start_piston_position
        while(depth < 0.3):
            rospy.sleep(1)
            piston_position -= 4
            set_piston_position(piston_position)
            
        rospy.loginfo("[Calibration] Piston Position = %i", piston_position)
        rospy.loginfo("[Calibration] depth = %f", depth)

    set_piston_position(0)
    

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
