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
start_piston_position = 1150
piston_position = 0

## ToDo : service reset zero pression + depth_set_point

def callback_fusionu_depth(data):
    global depth, velocity
    depth = data.depth
    velocity = data.velocity

# def callback_piston(data):
#     global piston_position
#     piston_position = data.position

def set_piston_position(target):
    try:
        resp = piston_position_set_point(target)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration] Fail to call Piston Set point");

def set_flash(enable):
    try:
        resp = flash_enable(enable)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration] Fail to call Flash Enable");

def set_zero_depth():
    try:
        resp = fusion_zero_depth()
        rospy.loginfo("[Calibration] Zero ")
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration] Fail to call Flash Enable");


def regulation_node():
    global flash_enable, fusion_zero_depth, start_piston
    global depth, velocity, piston_position, piston_position_set_point
    rospy.init_node('calibration_piston_node', anonymous=True)

    # sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusionu_depth ,queue_size=1)
    
    rospy.loginfo("[Calibration] Wait for Services")
    rospy.wait_for_service('/driver/piston/position')
    rospy.wait_for_service('/driver/piston/start')
    rospy.wait_for_service('/fusion/zero_depth')
    # rospy.wait_for_service('/driver/power/flash_led')

    piston_position_set_point = rospy.ServiceProxy('/driver/piston/position', PistonPosition)
    start_piston = rospy.ServiceProxy('/driver/piston/start', SetBool)
    fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Trigger)
    # flash_enable = rospy.ServiceProxy('/driver/power/flash_led', SetBool)

    time_start = rospy.Time().now()

    ######################################################################
    ########## Start the piston
    ######################################################################
    try:
        resp1 = start_piston(True)
        rospy.loginfo("[Calibration] Piston Started")
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration] Fail to call Piston start");
    

    try:
        resp2 = fusion_zero_depth()
        rospy.loginfo("[Calibration] Zero pressure done")
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration] Fail to call Fusion Zero");

    ######################################################################
    ########## Sink
    ######################################################################
    
    set_piston_position(start_piston_position)

    # Wait until depth is under 1.5m (sink)
    rospy.loginfo("[Calibration] Start Reaching 1.5m depth")
    while(depth<1.5):
        rospy.loginfo("[Calibration] depth = %f", depth)
        rospy.sleep(5.0)

    rospy.loginfo("[Calibration] 1.5m depth Reached")

    # Wait until depth stabilized to bottom
    set_flash(True)
    rospy.sleep(45.0)
    set_flash(False)

    # Save bottom depth
    bottom_depth = depth
    rospy.loginfo("[Calibration] bottom_depth = %f", bottom_depth)

    rospy.loginfo("[Calibration] Start Calibration 1")

    # Go up until 2 cm diff
    piston_position = start_piston_position
    while(bottom_depth - depth < 0.03):
        piston_position -= 4
        set_piston_position(piston_position)
        rospy.sleep(1)

    rospy.loginfo("[Calibration] Piston Position = %i", piston_position)
    rospy.loginfo("[Calibration] depth = %f", depth)

    # ## Do it 3 times near the point
    # for i in range(3):
    #     # set_flash(True)
    #     piston_position = min(1200, piston_position + 50)
    #     set_piston_position(piston_position)
    #     rospy.sleep(30.0)
    #     # set_flash(False)

    #     bottom_depth = depth
    #     while(bottom_depth - depth < 0.1):
    #         piston_position -= 1
    #         set_piston_position(piston_position)
    #         rospy.sleep(2)
    #     rospy.loginfo("[Calibration] Piston Position = %i", piston_position)
    #     rospy.loginfo("[Calibration] depth = %f", depth)

    set_piston_position(0)
    

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
