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

### Model
depth = 0.0
velocity = 0.0
g=9.81
V_drone = 0.00664 # To Update
m = 6.64
rho_eau = 1000.0
Cx=10.0
R_tube = (110.0/2.0)
S_Cx=(R_tube**2)*pi
# tick per turn : 24
# tige filetee : 1.75 mm
# 1 tick = 0.00175/24 = 0,000072917 m
# Diam piston = 50mm
tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*np.pi
coeff_Frottement = 0.5*Cx*S_Cx*rho_eau/m
coeff_acc_drone = g - g*V_drone*rho_eau/m
coeff_V_piston = g*rho_eau/m

### Depth & Velocity
velocity = 0
depth = 0

### Piston
piston_position = 0
offset_piston = 1130

### Asservissement
K = 50.0
depth_set_point = 0.0

## ToDo : service reset zero pression + depth_set_point

def callback_fusionu_depth(data):
    depth = data.depth
    velocity = data.velocity

def callback_piston(data):
    piston_position = data.position

def handle_depth_set_point(req):
    global depth_set_point
    depth_set_point = req.depth
    return DepthPointResponse()

def regulation_node():
    rospy.init_node('regulation_depth_node', anonymous=True)

    sub_piston = rospy.Subscriber("/driver/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/fusion/depth", DepthPose, callback_fusionu_depth ,queue_size=1)
    pub_debug = rospy.Publisher("regulation", RegulationDebug, queue_size=1)

    ser_depth_set_point = rospy.Service('depth_set_point', DepthPoint, handle_depth_set_point)

    rospy.wait_for_service('/driver/piston/position')
    rospy.wait_for_service('/driver/piston/start')
    rospy.wait_for_service('/fusion/zero_depth')
    piston_position_set_point = rospy.ServiceProxy('/driver/piston/position', PistonPosition)
    start_piston = rospy.ServiceProxy('/driver/piston/start', SetBool)
    fusion_zero_depth = rospy.ServiceProxy('fusion/zero_depth', Empty)

    delta_time = rospy.get_param('~delta_time', 1.0)
    sleep_time = rospy.Duration(1.0)

    time_start = rospy.Time().now()

    # Start the piston
    try:
        resp1 = start_piston(True)
    except rospy.ServiceException, e:
        rospy.logwarn("[Depth Regulation] Fail to call Piston start");

    try:
        resp2 = fusion_zero_depth()
    except rospy.ServiceException, e:
        rospy.logwarn("[Depth Regulation] Fail to call Fusion Zero");


    target_position = 0
    while not rospy.is_shutdown():
        
        V_piston = (piston_position-offset_piston)*tick_to_volume
        ## Command law
        # 0.00001*(-(g-g*((V_estime+V_piston)*rho_eau/m)-0.5*Cx*S_Cx*ddot*abs(ddot)*rho_eau/m)+100.0*ddot+(d-d0))
        if(abs(depth_set_point-depth)<0.2):
            K = 400.0
        else:
            K = 50.0

        target_position_tmp = target_position
        target_position += (0.00000001/tick_to_volume)*(-coeff_acc_drone - coeff_V_piston*V_piston - coeff_Frottement*velocity*abs(velocity) + K*velocity + 100*(depth_set_point - depth))

        target_position_offset = round(target_position + offset_piston)
        if(target_position_offset<0):
            target_position_offset = 0
            target_position = target_position_tmp
        if(target_position_offset>1200):
            target_position_offset = 1200
            target_position = target_position_tmp

        if((rospy.Time.now() - time_start).to_sec() > 600):
            target_position_offset = 0

        try:
            resp = piston_position_set_point(target_position_offset)
        except rospy.ServiceException, e:
            rospy.logwarn("[Depth Regulation] Fail to call Piston set point");

        msg_regulation = RegulationDebug()
        msg_regulation.volume_piston = V_piston
        msg_regulation.depth_set_point = depth_set_point
        msg_regulation.K = K
        msg_regulation.target_position = target_position
        msg_regulation.offset_piston = offset_piston
        msg_regulation.target_position_offset = target_position_offset
        pub_debug.publish(msg_regulation)

        rospy.sleep(sleep_time)

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
