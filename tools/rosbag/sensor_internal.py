import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
# from gmplot import gmplot
import yaml

bag = rosbag.Bag('2018-06-15-17-36-53.bag', 'r')
print(bag)

startTime = rospy.Time.from_sec(bag.get_start_time() + 130)

time = []
pressure = []
temperature = []
humidity = []

for topic, msg, t in bag.read_messages(topics="/sensor_internal", start_time=startTime):
	if(msg.pressure > 500):
		time.append(t.to_sec())
		pressure.append(msg.pressure/(msg.temperature + 273.15))
		temperature.append(msg.temperature)
		humidity.append(msg.humidity)

bag.close()

plt.subplot(311)
plt.ylabel("pressure")
plt.plot(time,pressure)

plt.subplot(312)
plt.ylabel("temperature")
plt.plot(time,temperature)

plt.subplot(313)
plt.ylabel("humidity")
plt.plot(time,humidity)

plt.show()


# info_dict = yaml.load(Bag('2018-06-15-17-36-53.bag', 'r')._get_yaml_info())