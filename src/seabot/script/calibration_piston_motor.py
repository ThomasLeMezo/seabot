#!/usr/bin/env python

import rospy

from seabot_piston_driver.msg import PistonState
from seabot_fusion.msg import DepthPose
from seabot_piston_driver.msg import *

from std_srvs.srv import *
from seabot_piston_driver.srv import *
from seabot_power_driver.srv import *

import numpy as np
from math import *

depth = 0

### Piston
piston_position_start = 1100
piston_position = 0
piston_speed_start = 10
speed_mini = 10

## ToDo : service reset zero pression + depth_set_point

def callback_fusion_depth(data):
    global depth, velocity
    depth = data.depth
    velocity = data.velocity

def callback_piston(data):
    global piston_position
    piston_position = data.position

def set_piston_position(target):
    global piston_position_pub
    position_msg = PistonPosition()
    position_msg.position = target
    piston_position_pub.publish(position_msg)

def set_flash(enable):
    global flash_enable
    try:
        resp = flash_enable(enable)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Flash Enable");

def set_flash_speed(speed):
    global flash_speed
    try:
        resp = flash_speed(round(speed*10.0))
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Flash Speed");

def set_zero_depth():
    global fusion_zero_depth
    try:
        resp = fusion_zero_depth()
        rospy.loginfo("[Calibration_Piston_Motor] Zero ")
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Flash Enable");

def set_piston_speed(val):
    global speed_motor
    try:
        resp = speed_motor(val, val)
        rospy.loginfo("[Calibration_Piston_Motor] Set Piston Speed %i ", val)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Piston Speed");


def regulation_node():
    global flash_enable, fusion_zero_depth, start_piston, piston_position_pub
    global depth, velocity, piston_position, piston_position_set_point
    global fusion_zero_depth, speed_motor, flash_speed, flash_enable
    global speed_mini, piston_speed_start, piston_position_start
    rospy.init_node('calibration_piston_node', anonymous=True)

    sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusion_depth ,queue_size=1)

    piston_position_pub = rospy.Publisher('/driver/piston/position', PistonPosition, queue_size=1)
    
    rospy.loginfo("[Calibration] Wait for Services")
    rospy.wait_for_service('/fusion/zero_depth')
    rospy.wait_for_service('/driver/power/flash_led')
    rospy.wait_for_service('/driver/piston/speed')

    fusion_zero_depth = rospy.ServiceProxy('/fusion/zero_depth', Trigger)
    flash_enable = rospy.ServiceProxy('/driver/power/flash_led', SetBool)
    flash_speed = rospy.ServiceProxy('/driver/power/flash_led_period', FlashSpeed)
    speed_motor = rospy.ServiceProxy('/driver/piston/speed', PistonSpeed)

    time_start = rospy.Time().now()

    ######################################################################
    ########## Main
    ######################################################################
    
    rospy.sleep(3.0)
    set_zero_depth()
    rospy.sleep(3.0)
    set_piston_speed(piston_speed_start)


    for depth_ref in range(1, 17, 3):
        rospy.loginfo("[CALIBRATION_PISTON] Start new sequence at %f m", depth_ref)
        
        # Reset piston position
        set_piston_speed(piston_speed_start)
        set_piston_position(piston_position_start)
        while(abs(piston_position-piston_position_start)>2):
            rospy.sleep(1)
            set_piston_position(piston_position_start)
        rospy.loginfo("[CALIBRATION_PISTON] Start Piston position reached")

        # Wait to reach desired depth
        
        while(abs(depth-depth_ref)>0.2):
            rospy.sleep(0.2)
            set_piston_position(piston_position_start)
        rospy.loginfo("[CALIBRATION_PISTON] Desired Depth reached")

        set_flash_speed(0.3)
        set_flash(True)
        rospy.sleep(3)
        set_flash(False)

        # Piston calibration
        set_piston_position(0)
        for piston_speed in range(speed_mini, 50, 5):
            piston_position_tmp = piston_position
            set_piston_speed(piston_speed)
            rospy.sleep(5.0) # Sleep for 5s
            if(abs(piston_position-piston_position_tmp)>10): # 2 tick /s
                rospy.loginfo("[CALIBRATION_PISTON] Speed = %i", piston_speed)
                speed_mini = piston_speed
                break

        set_flash_speed(1.0)
        set_flash(True)
        rospy.sleep(3)
        set_flash(False)

    set_piston_speed(40)
    set_piston_position(0)
    set_flash_speed(3.0)
    set_flash(True)
    rospy.sleep(15)
    set_flash(False)

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
