#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('/piston/cmd_position_piston', UInt16, queue_size=1)
    rospy.init_node('up_down_piston_node', anonymous=True)

    rate = rospy.Rate(1.0/30.0)
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
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass