#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pressure_89bsd_driver.msg import *
from kellerLD import KellerLD

def talker():
    
    frequency = rospy.get_param("/pressure_sensor_ext/frequency", 5.0)
    rospy.init_node('kellerLD', anonymous=True)
    pressure = rospy.Publisher('sensor_external', PressureBsdData, queue_size=10)
    rate = rospy.Rate(frequency) # 50hz
    while not rospy.is_shutdown():
        sensor = KellerLD()
        sensor.init()
        sensor.read()
        msg = PressureBsdData()
        msg.pressure = sensor.pressure()
        msg.temperature = sensor.temperature()
#        print(("pressure: %7.4f bar\ttemperature: %0.2f C") % (sensor.pressure(), sensor.temperature()))
        #pressureVal = "pressure = %s" % rospy.get_time()
        #rospy.loginfo(msg)
        pressure.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
