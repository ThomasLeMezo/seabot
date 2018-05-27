#!/bin/python3
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rosbag

# from gmplot import gmplot
import yaml


bag = rosbag.Bag('2018-06-15-17-36-53.bag', 'r')
print(bag)

startTime = rospy.Time.from_sec(bag.get_start_time() + 130)

time = []
pressure = []
temperature = []

for topic, msg, t in bag.read_messages(topics="/sensor_external", start_time=startTime):
	if(msg.temperature > 0 and msg.temperature < 50 and msg.pressure < 1.3 and msg.pressure > 0):
		time.append(t.to_sec() - bag.get_start_time())
		pressure.append(msg.pressure)
		temperature.append(msg.temperature)

bag.close()

plt.subplot(211)
plt.ylabel("pressure")
plt.plot(time,signal.medfilt(pressure, 3))

plt.subplot(212)
plt.ylabel("temperature")
plt.plot(time,signal.medfilt(temperature, 3))

plt.show()


# info_dict = yaml.load(Bag('2018-06-15-17-36-53.bag', 'r')._get_yaml_info())