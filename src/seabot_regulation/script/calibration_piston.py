#!/usr/bin/env python

import rospy

from piston_driver.msg import PistonState
from std_msgs.msg import Float64MultiArray
from fusion.msg import DepthPose
from seabot_regulation.msg import RegulationDebug

from piston_driver.srv import PistonPosition
from std_srvs.srv import *
from seabot_regulation.srv import *

import numpy as np
from math import *

depth = 0

### Piston
start_piston_position = 1150

## ToDo : service reset zero pression + depth_set_point

def callback_fusionu_depth(data):
    depth = data.depth
    velocity = data.velocity

def callback_piston(data):
    piston_position = data.position

def set_piston_position(target):
    try:
        resp = piston_position_set_point(target)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration] Fail to call Piston set point");

def set_flash(enable):
    try:
        resp = flash_enable(enable)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration] Fail to call Flash Enable");


def regulation_node():
    global piston_position_set_point
    rospy.init_node('regulation_depth_node', anonymous=True)

    sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusionu_depth ,queue_size=1)
    
    rospy.wait_for_service('/driver/piston/position')
    rospy.wait_for_service('/driver/piston/start')
    rospy.wait_for_service('/fusion/zero_depth')
    rospy.wait_for_service('/driver/power/flash_led')

    piston_position_set_point = rospy.ServiceProxy('/driver/piston/position', PistonPosition)
    start_piston = rospy.ServiceProxy('/driver/piston/start', SetBool)
    fusion_zero_depth = rospy.ServiceProxy('fusion/zero_depth', Empty)
    flash_enable = rospy.ServiceProxy('/driver/power/flash_led', SetBool)

    delta_time = rospy.get_param('~delta_time', 1.0)
    sleep_time = rospy.Duration(1.0)

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
        rospy.sleep(5.0)

    # Wait until depth stabilized to bottom
    rospy.sleep(25.0)
    set_flash(True)
    rospy.sleep(5.0)
    set_flash(False)

    # Save bottom depth
    bottom_depth = depth

    # Go up until 2 cm diff
    piston_position = start_piston_position
    while(abs(bottom-depth - depth) < 0.02):
        piston_position -= 1
        set_piston_position(piston_position)
        rospy.sleep(5)

    rospy.loginfo("[Calibration] Piston Position = %i", piston_position)

    ## Do it 3 times near the point
    for i in range(3):
        set_flash(True)
        piston_position = max(1200, piston_position + 50)
        set_piston_position(piston_position)
        rospy.sleep(15.0)
        set_flash(False)

        bottom_depth = depth
        while(abs(bottom-depth - depth) < 0.02):
            piston_position -= 1
            set_piston_position(piston_position)
            rospy.sleep(5)
        rospy.loginfo("[Calibration] Piston Position = %i", piston_position)

    set_piston_position(0)
    

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
