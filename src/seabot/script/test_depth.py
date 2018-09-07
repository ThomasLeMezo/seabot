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

depth_limit = 18.0
piston_position_sink = 2300
emergency = True

piston_speed = 15 # default 15
piston_speed_max = 45 # max 50
velocity_min = 10.0 # 10 ticks/s

## ToDo : service reset zero pression + depth_set_point

def callback_fusion_depth(data):
    global depth, velocity
    depth = data.depth
    velocity = data.velocity

def callback_emergency(req):
    if(req.data):
        emergency=True
    else:
        emergency=False
    return SetBoolResponse(True, "")

def set_piston_position(target):
    global piston_position_pub
    position_msg = PistonPosition()
    position_msg.position = target
    piston_position_pub.publish(position_msg)

def set_piston_speed(val):
    global speed_motor
    try:
        resp = speed_motor(val, val)
        rospy.loginfo("[Calibration_Piston_Motor] Set Piston Speed %i ", val)
    except rospy.ServiceException, e:
        rospy.logwarn("[Calibration_Piston_Motor] Fail to call Piston Speed");

def callback_piston(data):
    global piston_position
    piston_position = data.position

def callback_piston_velocity(data):
    global piston_velocity    
    piston_velocity = data.velocity

def regulation_node():
    global depth, velocity, depth_limit, piston_position, piston_velocity
    global piston_position_pub
    global speed_motor
    rospy.init_node('regulation_depth_limit', anonymous=True)

    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusion_depth ,queue_size=1)
    sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_piston = rospy.Subscriber("/driver/piston/velocity", PistonVelocity, callback_piston_velocity, queue_size=1)

    piston_position_pub = rospy.Publisher('/driver/piston/position', PistonPosition, queue_size=1)
    
    emergency_service = rospy.Service('emergency', SetBool, callback_emergency)

    rospy.wait_for_service('/driver/piston/speed')
    speed_motor = rospy.ServiceProxy('/driver/piston/speed', PistonSpeed)

    ######################################################################
    ########## Main
    ######################################################################
    
    rospy.loginfo("[Test_Depth] Start Ok")

    while(depth < depth_limit):
        if(emergency):
            break
        else:
            set_piston_position(piston_position_sink)
        rospy.sleep(1.0)

    set_piston_position(0)
    rospy.sleep(4.0)

    while(depth < 2.0):
        set_piston_position(0)
        rospy.sleep(1.0)
        if(abs(piston_velocity) < velocity_min and piston_speed < piston_speed_max):
            piston_speed += 5.0
            set_piston_speed(min(piston_speed, piston_speed_max))
            rospy.loginfo("[Test_Depth] Increase speed to " + min(piston_speed, piston_speed_max))

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
