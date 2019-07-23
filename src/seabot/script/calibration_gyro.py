#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import time

gyro_x = []
gyro_y = []
gyro_z = []

def callback_imu(data):
    global gyro_x, gyro_y, gyro_z
    gyro_x.append(data.angular_velocity.x)
    gyro_y.append(data.angular_velocity.y)
    gyro_z.append(data.angular_velocity.z)

def calibration_node():
    global save_data, acc_min, acc_max
    rospy.init_node('calibration_imu_node', anonymous=True)

    sub_pressure = rospy.Subscriber("/driver/imu", Imu, callback_imu ,queue_size=1)
    for i in range(60, 0, -1):
        print(i, end='\r')
        time.sleep(1)
    print(np.mean(np.array([gyro_x, gyro_y, gyro_z]), 1))


if __name__ == '__main__':
    try:
        calibration_node()
    except rospy.ROSInterruptException:
        pass
