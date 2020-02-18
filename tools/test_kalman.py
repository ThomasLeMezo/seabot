#!/usr/bin/env python
# license removed for brevity
import rospy
from seabot_fusion.msg import DepthPose
from seabot_piston_driver.msg import PistonState

def talker():
    pub_depth = rospy.Publisher('/fusion/depth', DepthPose, queue_size=10)
    pub_state = rospy.Publisher('/driver/piston/state', PistonState, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    t=0.
    while not rospy.is_shutdown():
        
        depthPose = DepthPose()
        pistonState = PistonState()

        depthPose.stamp = rospy.Time(t)#rospy.get_rostime()
        pistonState.stamp = rospy.Time(t)#rospy.get_rostime()

        depthPose.depth = 1.0
        pistonState.position = 1000.0

        pub_depth.publish(depthPose)
        pub_state.publish(pistonState)
        
        rate.sleep()
        raw_input("Press Enter to continue...")
        t+=1./5.

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
