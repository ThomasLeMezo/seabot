#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from std_srvs.srv import *

def talker():
    rospy.init_node('up_down_piston_node', anonymous=True)
    pub = rospy.Publisher('/piston/position_set_point', UInt16, queue_size=1)

    delta_time = rospy.get_param('~delta_time', 60.0)
    sleep_time = rospy.Duration(delta_time)

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
        if(down == True):
            target_position=0
            down=False
        else:
            target_position=1200
            down=True

        pub.publish(target_position)
        rospy.sleep(sleep_time)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
