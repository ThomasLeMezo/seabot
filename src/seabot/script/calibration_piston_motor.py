#!/usr/bin/env python

import rospy

from seabot_piston_driver.msg import PistonState
from std_msgs.msg import Float64MultiArray
from seabot_fusion.msg import DepthPose
from seabot_depth_regulation.msg import RegulationDebug
from seabot_piston_driver.msg import *

from std_srvs.srv import *
from seabot_depth_regulation.srv import *
from seabot_piston_driver.srv import *

import numpy as np
from math import *

depth = 0

### Piston
start_piston_position = 1000
piston_position = 0
piston_speed_start = 30

## ToDo : service reset zero pression + depth_set_point

def callback_fusion_depth(data):
    global depth, velocity
    depth = data.depth
    velocity = data.velocity

def set_piston_position(target):
    global piston_position_pub
    position_msg = PistonPosition()
    position_msg.position = target
    piston_position_pub.publish(position_msg)

def set_flash(enable):
    try:
        resp = flash_enable(enable)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Flash Enable");

def set_zero_depth():
    try:
        resp = fusion_zero_depth()
        rospy.loginfo("[Calibration_Piston_Motor] Zero ")
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Flash Enable");

def set_piston_speed(val):
    try:
        resp = speed_motor(val, val)
        rospy.loginfo("[Calibration_Piston_Motor] Set Piston Speed %i ", val)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Piston Speed");


def regulation_node():
    global flash_enable, fusion_zero_depth, start_piston, piston_position_pub
    global depth, velocity, piston_position, piston_position_set_point
    rospy.init_node('calibration_piston_node', anonymous=True)

    # sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusion_depth ,queue_size=1)
    piston_position_pub = rospy.Publisher('/driver/piston/position', PistonPosition, queue_size=1)
    
    rospy.loginfo("[Calibration] Wait for Services")
    rospy.wait_for_service('/fusion/zero_depth')
    rospy.wait_for_service('/driver/power/flash_led')
    rospy.wait_for_service('/driver/piston/speed')

    fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Trigger)
    flash_enable = rospy.ServiceProxy('/driver/power/flash_led', SetBool)
    speed_motor = rospy.ServiceProxy('/driver/piston/speed', PistonSpeed)

    time_start = rospy.Time().now()

    ######################################################################
    ########## Main
    ######################################################################
    
    rospy.sleep(3.0)
    set_zero_depth()
    rospy.sleep(3.0)
    set_piston_speed(piston_speed_start)

    while(depth < 15.0):    
        set_piston_position(start_piston_position)
        rospy.sleep(1.0)

    while(depth > 1.0):
        set_piston_position(0)
        set_flash(True)
        rospy.sleep(3.0)
        set_flash(False)
        rospy.sleep(7.0)
        if(piston_speed_start<110):
            piston_speed_start+=10
        set_piston_speed(piston_speed_start)

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
