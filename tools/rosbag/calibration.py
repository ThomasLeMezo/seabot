#!/bin/python3
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor

import rospy
import rosbag

# from gmplot import gmplot
import yaml
from math import*


# bag = rosbag.Bag('2018-06-16-07-11-18.bag', 'r')
bag = rosbag.Bag('2018-06-16-07-29-11.bag', 'r')

print(bag)

startTime = rospy.Time.from_sec(bag.get_start_time())# + rospy.Duration(600)
end_time = rospy.Time.from_sec(bag.get_end_time())# + rospy.Duration(100)

time_piston = []
position = []
position_set_point = []
time_depth = []
depth =[]
velocity =[]

for topic, msg, t in bag.read_messages(topics=["/driver/piston/state", "/fusion/depth"], start_time=startTime, end_time=end_time):
	if(topic=="/driver/piston/state"):
		time_piston.append((t-startTime).to_sec())
		position.append(msg.position)
		position_set_point.append(msg.position_set_point)
	if(topic=="/fusion/depth"):
		if(abs(msg.velocity)<1.0):
			time_depth.append((t-startTime).to_sec())
			depth.append(msg.depth)
			velocity.append(msg.velocity)

bag.close()

plt.subplot(211)
plt.ylabel("depth")
plt.plot(time_depth,depth)
plt.plot(time_depth,velocity, 'r')

plt.subplot(212)
plt.ylabel("piston")
plt.plot(time_piston,position)
plt.plot(time_piston,position_set_point, 'r')

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, facecolor='#FFFFCC')
ax.plot(time_depth, depth)
ax2 = ax.twinx()
ax2.plot(time_piston, position, 'r')
cursor = Cursor(ax2, useblit=True, color='red', linewidth=1)

plt.show()