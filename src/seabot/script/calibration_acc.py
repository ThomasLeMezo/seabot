#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import time

acc_min = np.array([10., 10., 10.])
acc_max = np.array([-10., -10., -10.])
save_data = False


def callback_imu(data):
    global save_data, acc_min, acc_max
    if(save_data):
        acc_min = np.minimum(acc_min, np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]))
        acc_max = np.maximum(acc_max, np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]))

def calibration_node():
    global save_data, acc_min, acc_max
    rospy.init_node('calibration_imu_node', anonymous=True)

    sub_pressure = rospy.Subscriber("/driver/imu", Imu, callback_imu ,queue_size=1)

    while not rospy.is_shutdown():
        raw_input()

        save_data = True
        time.sleep(5)
        save_data = False
        print(acc_min)
        print(acc_max)


if __name__ == '__main__':
    try:
        calibration_node()
    except rospy.ROSInterruptException:
        pass
