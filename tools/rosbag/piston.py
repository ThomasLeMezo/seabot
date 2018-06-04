#!/bin/python3
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rosbag

# from gmplot import gmplot
import yaml


bag = rosbag.Bag('2018-06-15-20-33-53.bag', 'r')
print(bag)

startTime = rospy.Time.from_sec(bag.get_start_time())

time = []
position = []
position_set_point = []

for topic, msg, t in bag.read_messages(topics="/piston/state", start_time=startTime):
	time.append(t.to_sec())
	position.append(msg.position)
	position_set_point.append(msg.position_set_point)

bag.close()

plt.ylabel("position")
plt.plot(time,position)
plt.plot(time,position_set_point, 'r')

plt.show()


# info_dict = yaml.load(Bag('2018-06-15-17-36-53.bag', 'r')._get_yaml_info())