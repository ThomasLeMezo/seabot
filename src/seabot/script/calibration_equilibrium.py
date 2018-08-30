#!/usr/bin/env python

import rospy

from seabot_piston_driver.msg import PistonState
from std_msgs.msg import Float64MultiArray
from seabot_fusion.msg import DepthPose
from seabot_depth_regulation.msg import RegulationDebug
from seabot_piston_driver.msg import PistonPosition

from std_srvs.srv import *
#from seabot_depth_regulation.srv import *

import numpy as np
from math import *

### Parameters
margin_depth = 0.9
start_piston_position = 2000

### Variables
depth = 0
piston_position = 0
piston_position_set_point = 0

### Piston tick to depth
tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*np.pi
piston_surface = np.pi*(0.03/2.0)**2

## Note
# V = 0.03*np.pi*((0.03/2.0)**2) ## 3 cm from top
# V/tick_to_volume => 148 # tick to top

def callback_fusion_depth(data):
    global depth, velocity
    depth = data.depth
    velocity = data.velocity

def callback_piston_state(data):
    global piston_position
    piston_position = data.position

def set_piston_position(target):
    global piston_position_pub
    position_msg = PistonPosition()
    position_msg.position = target
    piston_position_pub.publish(position_msg)

def set_flash(enable):
    try:
        resp = flash_enable(enable)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_piston] Fail to call Flash Enable");

def flash(time):
    set_flash(True)
    rospy.sleep(time)
    set_flash(False)

def set_zero_depth():
    try:
        resp = fusion_zero_depth()
        rospy.loginfo("[Calibration_piston] Zero ")
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_piston] Fail to call Flash Enable");

def reset_position():
    global piston_position, start_piston_position
    while(abs(piston_position-start_piston_position)>2.0):
        set_piston_position(start_piston_position)
        rospy.sleep(1.0)

def regulation_node():
    global flash_enable, fusion_zero_depth, start_piston, piston_position_pub
    global depth, velocity, piston_position, piston_position_set_point
    rospy.init_node('calibration_piston_node', anonymous=True)

    # sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusion_depth ,queue_size=1)
    sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston_state ,queue_size=1)
    piston_position_pub = rospy.Publisher('/driver/piston/position', PistonPosition, queue_size=1)
    
    rospy.loginfo("[Calibration] Wait for Services")
    rospy.wait_for_service('/fusion/zero_depth')
    rospy.wait_for_service('/driver/power/flash_led')

    fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Trigger)
    flash_enable = rospy.ServiceProxy('/driver/power/flash_led', SetBool)

    time_start = rospy.Time().now()

    ######################################################################
    ########## Main
    ######################################################################
    
    rospy.loginfo("[Calibration_piston] Move piston to start position")
    reset_position()
    rospy.sleep(3.0)
    rospy.loginfo("[Calibration_piston] Zero depth")
    flash(3.0)
    rospy.sleep(10.0)
    set_zero_depth()
    
    for i in range(3):
        rospy.loginfo("[Calibration] ##### Calibration %i", i)
        ## Reset piston position (piston move)
        rospy.loginfo("[Calibration_piston] Wait Piston position")
        while(abs(piston_position-start_piston_position)>5.0):
            set_piston_position(start_piston_position)
            rospy.sleep(1.0)

        rospy.sleep(10.0)
        ## Wait depth reach (in range for at least 1s)
        rospy.loginfo("[Calibration_piston] Wait stabilized depth")
        depth_last = depth+5.0 # Init with different values
        while(depth > 0.05 or depth < 0.0 or depth_last > 0.05 or depth_last < 0.0):
            depth_last = depth
            set_piston_position(start_piston_position)
            rospy.sleep(1.0)

        rospy.loginfo("[Calibration_piston] Move piston")
        flash(3.0)
        piston_position_set_point = start_piston_position
        rospy.sleep(15.0)

        # depth*piston_surface == abs(start_piston_position-start_piston_position)*tick_to_volume

        while(depth < 0.2):
            # while(margin_depth*depth*piston_surface < abs(start_piston_position-start_piston_position)*tick_to_volume):
            rospy.sleep(0.2)
            piston_position_set_point += 0.2
            set_piston_position(np.floor(piston_position_set_point))
            
        rospy.loginfo("[Calibration] Piston Position Set Point = %i", piston_position_set_point)
        rospy.loginfo("[Calibration] Piston Position = %i", piston_position)
        rospy.loginfo("[Calibration] depth = %f", depth)

    set_piston_position(0)
    

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
