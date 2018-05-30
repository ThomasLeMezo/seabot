#!/usr/bin/env python

import rospy
from piston_driver.msg import *
from pressure_89bsd_driver.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
import numpy as np

depth = 0.0
velocity = 0.0
g=9.81
V_drone = 0.00664 # To Update
m = 6.64
rho_eau = 1000.0
# tick per turn : 24
# tige filetee : 1.75 mm
# 1 tick = 0.00175/24 = 0,000072917 m
# Diam piston = 50mm
tick_to_volume = (1.75e-3/24)*((0.05/2.0)**2)*np.pi

piston_position = 0

depth_set_point = 1.25

coeff_Frottement = 0.5*Cx*S_Cx*rho_eau/m
coeff_acc_drone = g*V_drone*rho_eau/m - g
coeff_V_piston = g*rho_eau/m

def callback_pressure(data):
    pressure = data.pressure
    pressure_velocity = data.pressure_velocity

def callback_pressure(data):
    piston_position = data.position

def regulation_node():
    rospy.init_node('regulation_depth_node', anonymous=True)
    pub = rospy.Publisher("/piston/position_set_point", UInt16, queue_size=1)
    sub_piston = rospy.Subscriber("/piston/state", PistonState, callback_piston, queue_size=1)
    sub_pressure = rospy.Subscriber("/sensor_external", bsdData, callback_pressure ,queue_size=1)

    freq = rospy.get_param('frequency', 1.0)
    rate = rospy.Rate(freq)

    # Start the piston

    rospy.wait_for_service('piston/start')
        try:
            start_piston = rospy.ServiceProxy('piston/start', SetBool)
            resp1 = start_piston(true)
        except rospy.ServiceException, e:
            rospy.logwarn("[Up and Down] Fail to call Piston start");

    down = True
    target_position = 0
    while not rospy.is_shutdown():
        
        V_piston = piston_position*tick_to_volume
        ## Command law
        target_position += np.sign(coeff_acc_drone + coeff_V_piston*V_piston - coeff_Frottement*velocity*abs(velocity) + 2*velocity + (depth_set_point-depth))

        if(target_position<0):
            target_position = 0
        if(target_position>1200):
            target_position = 1200

        pub.publish(target_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        regulation_node()
    except rospy.ROSInterruptException:
        pass
