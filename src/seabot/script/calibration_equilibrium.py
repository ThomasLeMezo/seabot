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

### Piston speed ###
dt_speed = 0.2
speed_table = [10.0, 5.0, 1.0]

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
    rospy.wait_for_service('/driver/power/flash')

    fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Trigger)
    flash_enable = rospy.ServiceProxy('/driver/power/flash', SetBool)

    time_start = rospy.Time().now()

    ######################################################################
    ########## Main
    ######################################################################
    
    rospy.sleep(10.0)
    rospy.loginfo("[Calibration_piston] Zero depth")
    flash(3.0)
    set_zero_depth()
    
    k_speed = 0
    for i in range(3):
        rospy.loginfo("[Calibration] ##### Calibration %i", i)
        if(i<len(speed_table)):
            k_speed = i

        ################# Set piston position to Start (piston move) #################
        rospy.loginfo("[Calibration_piston] Wait Piston position")
        while(abs(piston_position-start_piston_position)>5.0):
            set_piston_position(start_piston_position)
            rospy.sleep(1.0)
        rospy.sleep(15.0) # Wait stabilisation

        ################# Start Diving #################
        rospy.loginfo("[Calibration_piston] Start Diving")
        flash(3.0)
        piston_position_set_point = start_piston_position
        while(depth < 0.2):
            # while(margin_depth*depth*piston_surface < abs(start_piston_position-start_piston_position)*tick_to_volume):
            rospy.sleep(dt_speed)
            piston_position_set_point += speed_table[k_speed]/dt_speed
            set_piston_position(np.floor(piston_position_set_point))
            
        rospy.loginfo("[Calibration] Piston Position Set Point = %i", piston_position_set_point)
        rospy.loginfo("[Calibration] Piston Position = %i", piston_position)
        rospy.loginfo("[Calibration] depth = %f", depth)

        ################# Return to surface #################
        set_piston_position(0)
        while(depth > 0.2):
            set_piston_position(0)
            rospy.sleep(1.0)

    set_piston_position(0)
    flash(3.0)
    

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
