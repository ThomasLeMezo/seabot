from math import *
import numpy as np
import matplotlib.pyplot as plt

g = 9.81
rho_eau = 1025.0 # kg/m3

########## Drone characteristics ##########

C_f = 0.043
equilibrium_tick = 2150
tick_compression = 10.0 # ticks per meters

tick_to_volume = (1.75e-3/48.0)*((0.05/2.0)**2)*np.pi

speed_in = 32.0 # tick/s
speed_out = 32.0 # tick/s

piston_tick_max = 2400
piston_tick_min = 0

volume_compression = -tick_compression*tick_to_volume # in delta_V / m

########## Drone regulation ##########
C_f_estim = C_f
# K_velocity = 120.0
# K_acc = 3000.0
K_e = 1.0
K_velocity = 120.0
# K_acc = 3000.0
# K_i = 3000.0

K_acc = 0.0
K_i = 0.0

K_factor = 0.2
delta_t_regulation = 1.0 # sec
set_point_following = 10.0

equilibrium_tick_error = 0.0

########## State Vector ##########
# Initial : position, velocity, piston volume, tick
# x[0] depth
# x[1] velocity
# x[2] Piston volume (0.0 = equilibrium)
# x[3] tick
x = np.zeros(4)
x[0] = 0.0
x[1] = 0.0

x[3] = equilibrium_tick+(x[0]*volume_compression)/tick_to_volume
x[2] = -(x[3]-equilibrium_tick)*tick_to_volume
print("x[2] = ", x[2], "x[3] = ", x[3])

########## Mission ##########
# waypoints = []
# for d in range(1, 17, 2):
# 	waypoints.append([d, 60*60])
# for d in range(16, 0, -2):
# 	waypoints.append([d, 60*60])

# waypoints = [[5.0, 60*60], [40, 60*60], [0.0, 10*60]]
waypoints = [[5.0, 60*60]]
# waypoints = [[5.0, 60*60*1]]
# waypoints = [[0.0, 100]]
# waypoints = [[5.0, 60*60], [10.0, 60*60], [0.0, 60*60], [5.0, 60*60]]

########## Simulation ##########
dt=0.001
time_simulation = 0.0 # sec
for w in waypoints:
	time_simulation+=w[1]
next_time_waypoint = 0
nb_waypoint = 0

depth_seafloor = 50.0
i_acc=0.0

### Log
a_log = []
v_log = []
e_log = []
i_log = []
u_log = []

##############################################################

def euler(x, u, dt):
	global t
	# Note : u => cmd in tick (inverted from volume variation)
	# y=np.array(x)
	y=np.array(x)

	y[0] += dt*x[1]
	y[1] += dt*(-g*((x[2]+x[0]*volume_compression)*rho_eau) - (0.5*C_f*x[1]*abs(x[1])*rho_eau))

	# Simulation limits
	if(x[0]>=depth_seafloor and y[1]>=0.0):
		y[0] = x[0]
		y[1] = 0.0
	if(x[0]<=0 and y[1]<=0.0):
		y[0] = x[0]
		y[1] = 0.0

	if(u>=0):
		y[3] += min(speed_out*dt, u)
	else:
		y[3] -= min(speed_in*dt, abs(u))

	if(y[3]>piston_tick_max):
		y[3]=piston_tick_max
	elif(y[3]<piston_tick_min):
		y[3]=piston_tick_min

	# if tick increase => volume decrease
	y[2] = (-y[3]+equilibrium_tick)*tick_to_volume

	return y

def K_e_v(e):
	return max(0, -750.0*abs(e)+1000)

def control(set_point, x, tick_target, tick_target_compensated, dt):
	global a_log, v_log, e_log, i_acc

	d_noise = x[0]# + np.random.standard_normal()*0.5e-3 # noise around centimeter
	ddot_noise = x[1]# + np.random.standard_normal()*1e-3

	V_piston = -(x[3]-equilibrium_tick+equilibrium_tick_error)*tick_to_volume

	a = K_acc*(-g*((V_piston+d_noise*volume_compression)*rho_eau) - (0.5*C_f_estim*x[1]*abs(x[1])*rho_eau))
	e = K_e*(set_point-d_noise)
	v = K_velocity*ddot_noise
	# v = K_velocity*ddot_noise*K_e_v(e)
	i_acc += (set_point-d_noise)*tick_to_volume
	# i_acc = min(a/K_i, i_acc)*np.sign(i_acc)
	i=K_i*i_acc
	cmd = K_factor*delta_t_regulation*(-v+e-a+i)

	# a_log.append(a)
	v_log.append(-v)
	e_log.append(e)
	a_log.append(-a)
	i_log.append(i)

	if(abs(x[3]-tick_target_compensated)<set_point_following):
		return cmd+tick_target
	else:
		return tick_target

def set_point_depth():
	global set_point, t, waypoints, next_time_waypoint, nb_waypoint

	if(t>=next_time_waypoint and nb_waypoint<len(waypoints)):
		next_time_waypoint = t+waypoints[nb_waypoint][1]
		set_point = waypoints[nb_waypoint][0]
		# i_acc=0.0
		nb_waypoint+=1

##############################################################

result_x = []
result_tick_target_compensated = []
result_t = []
result_set_point = []

t=0.0
tick_target=x[3]
tick_target_compensated=tick_target
print("time simulation = ", time_simulation/(60.0*60.0), " hours")

print("k_max = ", int(time_simulation/dt))
for k in range(0, int(time_simulation/dt)):
	t+=dt
	## Set point
	set_point_depth()

	## Compute cmd
	if ((k % int((delta_t_regulation/dt))) == 0):
		if(set_point>0.2):
			tick_target = control(set_point, x, tick_target, tick_target_compensated, dt) # u=volume targeted
			tick_target_compensated = tick_target - x[0]*tick_compression
		else:
			tick_target_compensated = -speed_out
		u_log.append(tick_target_compensated)

	## Euler
	x = euler(x, round(tick_target_compensated-x[3]), dt)

	# print(y[1])

	## Save results
	result_x.append(x)
	result_tick_target_compensated.append(tick_target_compensated)
	# result_set_point.append(set_point)
	result_t.append(t)

################################################
############### 	Plots	####################
################################################

plt.figure(1, figsize=(15,7))
plt.subplot(311)
plt.ylabel('depth')
plt.plot(result_t, np.transpose(result_x)[0], 'r')
# plt.plot(result_t, np.transpose(result_set_point), 'b')

plt.subplot(312)
plt.ylabel('Ticks')
plt.plot(result_t, np.transpose(result_x)[3], 'b')

plt.subplot(313)
plt.ylabel('Volume')
plt.plot(result_t, np.transpose(result_x)[2])

plt.figure(2)
# plt.subplot(311)
# plt.ylabel('tick_target_compensated')
# plt.plot(result_t, result_tick_target_compensated, 'r')

plt.subplot(511)
plt.ylabel('e')
plt.plot(e_log, 'r')

plt.subplot(512)
plt.ylabel('v')
plt.plot(v_log, 'r')

plt.subplot(513)
plt.ylabel('a')
plt.plot(a_log, 'r')

plt.subplot(514)
plt.ylabel('i')
plt.plot(i_log, 'r')

plt.subplot(515)
plt.ylabel('ticks')
plt.plot(np.transpose(result_x)[3], 'r')

plt.show()

# try:
#   while True:
#     pass
# except KeyboardInterrupt:
#   pass


