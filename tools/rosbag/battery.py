import rosbag
import numpy as np
import matplotlib.pyplot as plt
from gmplot import gmplot
import yaml

bag = rosbag.Bag('2018-06-15-17-36-53.bag', 'r')
print(bag)

time = []
b1 = []
b2 = []
b3 = []
b4 = []

for topic, msg, t in bag.read_messages(topics="/battery"):
	time.append(t.to_sec())
	# print(msg)
	b1.append(msg.battery1)
	b2.append(msg.battery2)
	b3.append(msg.battery3)
	b4.append(msg.battery4)
	# data.append(msg.battery1)

bag.close()

plt.subplot(411)
plt.plot(time,b1)

plt.subplot(412)
plt.plot(time,b2)

plt.subplot(413)
plt.plot(time,b3)

plt.subplot(414)
plt.plot(time,b4)

plt.show()


# info_dict = yaml.load(Bag('2018-06-15-17-36-53.bag', 'r')._get_yaml_info())