from __future__ import print_function, division

import rospy
import rosbag
import yaml
import numpy as np

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ellipsoid_fit import ellipsoid_fit, ellipsoid_plot, data_regularize


# /driver/mag
time_mag = []
mag = []

def load_bag(filename):
	global mag_x, mag_y, mag_z
	bag = rosbag.Bag(filename, 'r')

	# print(bag)
	startTime = rospy.Time.from_sec(bag.get_start_time())# + rospy.Duration(600)
	end_time = rospy.Time.from_sec(bag.get_end_time())# + rospy.Duration(100)

	for topic, msg, t in bag.read_messages(topics=['/driver/mag'], start_time=startTime, end_time=end_time):
		if(topic=="/driver/mag"):
			time_mag.append((t-startTime).to_sec())
			mag.append([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
	bag.close()

if(len(sys.argv)<2):
	sys.exit(0)

load_bag(sys.argv[1])

data = np.array(mag)
data2 = data_regularize(data, divs=128)

center, radii, evecs, v = ellipsoid_fit(data2)

dataC = data - center.T
dataC2 = data2 - center.T

a, b, c = radii
r = (a * b * c) ** (1. / 3.) #preserve volume?
D = np.array([[r/a, 0., 0.], [0., r/b, 0.], [0., 0., r/c]])
#http://www.cs.brandeis.edu/~cs155/Lecture_07_6.pdf
#affine transformation from ellipsoid to sphere (translation excluded)
TR = evecs.dot(D).dot(evecs.T)
dataE = TR.dot(dataC2.T).T

print('ellipsoid_offset: [', center[0][0], ', ', center[1][0], ', ', center[2][0], ']')
print('ellipsoid_matrix0: [', TR[0][0], ', ', TR[0][1], ', ', TR[0][2], ']')
print('ellipsoid_matrix1: [', TR[1][0], ', ', TR[1][1], ', ', TR[1][2], ']')
print('ellipsoid_matrix2: [', TR[2][0], ', ', TR[2][1], ', ', TR[2][2], ']')

# np.savetxt('magcal_ellipsoid.txt', np.vstack((center.T, TR)))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#hack  for equal axes
ax.set_aspect('equal')
MAX = 1e-4
for direction in (-1, 1):
    for point in np.diag(direction * MAX * np.array([1, 1, 1])):
        ax.plot([point[0]], [point[1]], [point[2]], 'w')
        
#ax.scatter(dataC[:,0], dataC[:,1], dataC[:,2], marker='o', color='g')
ax.scatter(dataC2[:, 0], dataC2[:, 1], dataC2[:, 2], marker='o', color='b')
ax.scatter(dataE[:, 0], dataE[:, 1], dataE[:, 2], marker='o', color='r')

ellipsoid_plot([0, 0, 0], radii, evecs, ax=ax, plotAxes=True, cageColor='g')
ellipsoid_plot([0, 0, 0], [r, r, r], evecs, ax=ax, plotAxes=True, cageColor='orange')

#ax.plot([r],[0],[0],color='r',marker='o')
#ax.plot([radii[0]],[0],[0],color='b',marker='o')
#print (np.array([radii[0],0,0]).dot(transform)[0], r)

plt.show()