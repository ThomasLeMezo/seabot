#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

def talker():
    rospy.init_node('up_down_piston_node', anonymous=True)
    pub = rospy.Publisher('/piston/position_set_point', UInt16, queue_size=1)

    delta_time = rospy.get_param('~delta_time', 30.0)
    sleep_time = rospy.Duration(delta_time)

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