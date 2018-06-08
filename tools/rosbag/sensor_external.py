#!/bin/python3
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rosbag

# from gmplot import gmplot
import yaml


# bag = rosbag.Bag('2018-06-16-03-27-18.bag', 'r')
# bag = rosbag.Bag('2018-06-16-01-33-07.bag', 'r')
bag = rosbag.Bag('2018-06-16-07-29-11.bag', 'r')

print(bag)

startTime = rospy.Time.from_sec(bag.get_start_time())# + rospy.Duration(600)
end_time = rospy.Time.from_sec(bag.get_end_time()) #startTime + rospy.Duration(100)

time = []
pressure = []
temperature = []
pressure_velocity = []

for topic, msg, t in bag.read_messages(topics="/driver/sensor_external", start_time=startTime, end_time=end_time):
	if(msg.temperature > 0 and msg.temperature < 50 and msg.pressure < 6 and msg.pressure > 0):
		time.append(t.to_sec() - bag.get_start_time())
		pressure.append((msg.pressure-1.08)*10.0)
		temperature.append(msg.temperature)
		# pressure_velocity.append(msg.pressure_velocity)

bag.close()

plt.subplot(211)
plt.ylabel("depth")
plt.plot(time,signal.medfilt(pressure, 1))

plt.subplot(212)
plt.ylabel("temperature")
plt.plot(time,temperature)

# plt.subplot(313)
# plt.ylabel("depth_velocity")
# plt.plot(time,signal.medfilt(pressure_velocity, 3))

plt.show()


# info_dict = yaml.load(Bag('2018-06-15-17-36-53.bag', 'r')._get_yaml_info())