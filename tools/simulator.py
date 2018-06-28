from math import *
import numpy as np
import matplotlib.pyplot as plt

tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*np.pi

########## Drone characteristics ##########
g = 9.81
rho_eau = 1020.0 # kg/m3
m = 8.810 # kg
C_f = 0.05

speed_in = 11.0
speed_out = 1.0

delta_volume_piston = 300 * tick_to_volume

# Initial : position, velocity, piston volume
x = np.array([0.0, 0.0, -tick_to_volume * 300])

# Speed : [0.16, 0.18] m/s for delta of 440 ticks
# 2*440*tick_to_volume*9.81/(0.18**2)
# => [0.04 0.06]

########## Drone regulation ##########
# C_f_estim = C_f
C_f_estim = 0.02
K_velocity = 300.0
K_factor = 0.05
delta_t_regulation = 1.0 # sec

offset = +100 * tick_to_volume

########## Simulation ##########
dt=0.05
time_simulation = 2*120*60 # sec

depth_seafloor = 20.0

#################################################

def set_point_depth():
	global d0, t
	mod = (t/60.0)%120.0
	if(mod<60.0):
		d0=5.0
	elif(mod<120.0):
		d0=15.0

V = m/rho_eau
rho_eau_m = rho_eau /m
V_estime = V
print("m (volume) = ", V*rho_eau)
print("m = ", m)
print("offset = ", offset)
# print("tick_to_volume = ", tick_to_volume)

def f(x, u):
	# Note : u => cmd in tick (inverted from volume variation)
	y=np.zeros(3)
	y[0] = x[1]
	y[1] = g - g*((V+x[2])*rho_eau/m) - (0.5*C_f*x[1]*abs(x[1])*rho_eau/m)
	volume_target = -u*tick_to_volume

	if(volume_target < x[2]): # reduce volume => piston move in
		y[2] -= min(speed_in*tick_to_volume, abs(x[2]-volume_target))
	elif(volume_target > x[2]):
		y[2] += min(speed_out*tick_to_volume, abs(x[2]-volume_target))
	else:
		y[2] = 0
	return y

# 1s / Kv 50 Kf 10
# 1s / Kv 70 Kf 1

def control(d0, d, ddot, V_piston, u):
	global t_old, t

	d_noise = d+np.random.random_sample()*1.1e-2 # noise around centimeter

	a = -g*V_piston*rho_eau_m
	cmd = -K_factor*(t-t_old)*(-g*((V_piston+offset)*rho_eau/m)-0.5*C_f*ddot*abs(ddot)*rho_eau/m+K_velocity*ddot+(d-d0))

	t_old = t
	return cmd+u

result_x = []
result_u = []
result_t = []
result_d0 = []

t=0
t_old = t-dt
u=0

for k in range(0, int(time_simulation/dt)):
	t+=dt

	# Set point
	set_point_depth()

	## Compute cmd
	d = x[0]
	ddot = x[1]
	V_piston = x[2]
	if k % int((delta_t_regulation/dt)) == 0:
		u = control(d0, d, ddot, V_piston, u)
	u_round = round(u)

	## Euler
	dx = f(x, u_round)
	x = x+dx*dt

	## Simulation limits
	if(abs(x[2])>delta_volume_piston):
		x[2] = delta_volume_piston*np.sign(x[2])
	if(x[0]>depth_seafloor):
		x[0] = depth_seafloor
		x[1] = 0.0
	if(x[0]<0.0):
		x[0] = 0.0
		x[1] = 0.0

	## Save results
	result_x.append(x)
	result_u.append(u_round)
	result_d0.append(d0)
	result_t.append(t)

################################################
############### 	Plots	####################
################################################

plt.figure(1)
plt.subplot(311)
plt.ylabel('depth')
plt.plot(result_t, np.transpose(result_x)[0], 'r')
plt.plot(result_t, np.transpose(result_d0), 'b')

plt.subplot(312)
plt.ylabel('command (in tick)')
plt.plot(result_t, np.transpose(result_u), 'r')
plt.plot(result_t, -np.transpose(result_x)[2]/tick_to_volume, 'b')

plt.subplot(313)
plt.ylabel('speed')
plt.plot(result_t, np.transpose(result_x)[1])

plt.show()


