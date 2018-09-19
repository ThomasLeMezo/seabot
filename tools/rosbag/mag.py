import rospy
import rosbag
# from gmplot import gmplot
import yaml
import numpy as np

import sys
from load_data import *

# /driver/mag
time_mag = []
mag_x = []
mag_y = []
mag_z = []

def load_bag(filename):
	global mag_x, mag_y, mag_z
	bag = rosbag.Bag(filename, 'r')

	print(bag)
	startTime = rospy.Time.from_sec(bag.get_start_time())# + rospy.Duration(600)
	end_time = rospy.Time.from_sec(bag.get_end_time())# + rospy.Duration(100)

	for topic, msg, t in bag.read_messages(topics=['/driver/mag'], start_time=startTime, end_time=end_time):
		if(topic=="/driver/mag"):
			time_mag.append((t-startTime).to_sec())
			mag_x.append(msg.magnetic_field.x)
			mag_y.append(msg.magnetic_field.y)
			mag_z.append(msg.magnetic_field.z)

	bag.close()

if(len(sys.argv)<2):
	sys.exit(0)

load_bag(sys.argv[1])

# Write Numpy array
print(np.size(mag_x))
np.savetxt("mag_test.txt", np.transpose(np.array((mag_x, mag_y, mag_z))))