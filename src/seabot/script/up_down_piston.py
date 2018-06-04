#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from piston_driver.srv import *

def talker():
    rospy.init_node('up_down_piston_node', anonymous=True)

    rospy.wait_for_service('piston/position')
    rospy.wait_for_service('piston/start')
    piston_position = rospy.ServiceProxy('piston/position', PistonPosition)
    start_piston = rospy.ServiceProxy('piston/start', SetBool)

    delta_time = rospy.get_param('~delta_time', 180.0)
    sleep_time = rospy.Duration(delta_time)

    # Start the piston
    try:
        resp1 = start_piston(true)
    except rospy.ServiceException, e:
        rospy.logwarn("[Up and Down] Fail to call Piston start");

    down = True
    target_position = 0
    while not rospy.is_shutdown():
        try:
            if(down == True):
                resp = piston_position(0)
                down=False
            else:
                resp = piston_position(1200)
                down=True
        except rospy.ServiceException, e:
            rospy.logwarn("[Up and Down] Fail to call Piston position");

        rospy.sleep(sleep_time)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
