from math import *
import numpy as np
import matplotlib.pyplot as plt

tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*np.pi

########## Drone characteristics ##########
g = 9.81
rho_eau = 1020.0 # kg/m3
# C_f = 0.00005
# C_f = 0.1
C_f = 0.01
# 0.001 => 71
# 

speed_in = 5.0 #
speed_out = 5.0 # Check ok ?

piston_tick_max = 1200
piston_tick_min = 0
offset_tick = 700

# Initial : position, velocity, piston volume, tick
x = np.zeros(4)
x[3] = 700

# waypoints = []
# for d in range(1, 17, 2):
# 	waypoints.append([d, 60*60])
# for d in range(16, 0, -2):
# 	waypoints.append([d, 60*60])

# waypoints = [[5.0, 60*60], [15, 60*60], [0.0, 60*60]]
waypoints = [[5.0, 60*60*1]]

# waypoints = [[5.0, 60*60], [10.0, 60*60], [0.0, 60*60], [5.0, 60*60]]

# Speed : [0.16, 0.18] m/s for delta of 440 ticks
# 2*440*tick_to_volume*9.81/(0.18**2)
# => [0.04 0.06] => compressibility ???

########## Drone regulation ##########
# C_f_estim = C_f
C_f_estim = C_f
K_velocity = 600.0
K_acc = 0.0 # 10000 OK
K_e = 1.0
K_factor = 0.1
delta_t_regulation = 1.0 # sec
set_point_following = 10.0

########## Simulation ##########
dt=0.001
time_simulation = 0.0 # sec
for w in waypoints:
	time_simulation+=w[1]
next_time_waypoint = 0
nb_waypoint = 0

tick_compression = 18.0
volume_compression = tick_compression*tick_to_volume # in delta_V / m

# depth_seafloor = 20.0
depth_seafloor = 20.0
offset_error = 0.0


### Log
a_log = []
v_log = []
e_log = []
u_log = []

##############################################################

def euler(x, u, dt):
	# Note : u => cmd in tick (inverted from volume variation)
	# y=np.array(x)
	y=np.array(x)
	y[0] += dt*x[1]
	y[1] += dt*(-g*((x[2]-x[0]*volume_compression*tick_to_volume)*rho_eau) - (0.5*C_f*x[1]*abs(x[1])*rho_eau))

	# Simulation limits
	if(x[0]>=depth_seafloor and y[1]>=0.0):
		y[0] = x[0]
		y[1] = 0.0
	if(x[0]<=0 and y[1]<=0.0):
		y[0] = x[0]
		y[1] = 0.0

	if(x[3]<u):
		y[3] += min(speed_out, abs(u-x[3]))
	elif(x[3]>u):
		y[3] -= min(speed_in, abs(u-x[3]))

	if(y[3]>piston_tick_max):
		y[3]=piston_tick_max
	elif(y[3]<piston_tick_min):
		y[3]=piston_tick_min

	y[2] = -(y[3]-offset_tick)*tick_to_volume

	return y

def control(set_point, x, u, dt):
	global a_log, v_log, e_log

	d_noise = x[0] #+ np.random.standard_normal()*0.5e-3 # noise around centimeter
	ddot_noise = x[1] #+ np.random.standard_normal()*1e-3

	V_piston = -(x[3]-offset_tick+offset_error)*tick_to_volume

	# a = K_acc*(-g*(V_piston-d_noise*volume_compression*tick_to_volume)*rho_eau-0.5*C_f_estim*ddot_noise*abs(ddot_noise)*rho_eau)
	v = K_velocity*ddot_noise
	e = K_e*(set_point-d_noise)
	cmd = K_factor*delta_t_regulation*(-v+e)

	# a_log.append(a)
	v_log.append(v)
	e_log.append(e)

	if(abs(x[3]-u)<set_point_following):
		return cmd+u
	else:
		return u

def set_point_depth():
	global set_point, t, waypoints, next_time_waypoint, nb_waypoint

	if(t>=next_time_waypoint and nb_waypoint<len(waypoints)):
		next_time_waypoint = t+waypoints[nb_waypoint][1]
		set_point = waypoints[nb_waypoint][0]
		nb_waypoint+=1

##############################################################

result_x = []
result_u = []
result_t = []
result_set_point = []

t=0.0
u=x[3]
print("time simulation = ", time_simulation/(60.0*60.0), " hours")

print("k_max = ", int(time_simulation/dt))
for k in range(0, int(time_simulation/dt)):
	t+=dt
	# Set point
	set_point_depth()

	## Compute cmd
	if ((k % int((delta_t_regulation/dt))) == 0):
		if(set_point>0.2):
			u = control(set_point, x, u, dt) # u=volume targeted
			u_offset = u + x[0]*volume_compression
		else:
			u_offset = 0.0
		u_log.append(u_offset)

	## Euler
	x = euler(x, round(u_offset), dt)

	# print(y[1])

	## Save results
	result_x.append(x)
	result_set_point.append(set_point)
	result_t.append(t)

################################################
############### 	Plots	####################
################################################

plt.figure(1, figsize=(15,7))
# plt.subplot(311)
plt.ylabel('depth')
plt.plot(result_t, np.transpose(result_x)[0], 'r')
plt.plot(result_t, np.transpose(result_set_point), 'b')

# plt.subplot(312)
# plt.ylabel('Ticks')
# plt.plot(result_t, np.transpose(result_x)[3], 'b')

# plt.subplot(313)
# plt.ylabel('speed')
# plt.plot(result_t, np.transpose(result_x)[1])

# plt.figure(2)
# plt.subplot(311)
# plt.ylabel('u')
# plt.plot(u_log, 'r')

# plt.subplot(312)
# plt.ylabel('e')
# plt.plot(e_log, 'r')

# plt.subplot(313)
# plt.ylabel('v')
# plt.plot(v_log, 'r')

# plt.subplot(414)
# plt.ylabel('a')
# plt.plot(a_log, 'r')

plt.show()

# try:
#   while True:
#     pass
# except KeyboardInterrupt:
#   pass


