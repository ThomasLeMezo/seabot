from math import *
import numpy as np
import matplotlib.pyplot as plt

tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*np.pi

g = 9.81
rho_eau = 1000.0 # kg/m3
m = 6.855 # kg

# R_tube = 0.110/2.0
# L_tube = 0.70
# V = (R_tube**2)*pi * L_tube + (0.03/2.0)**2*pi*0.30 # Antenna
V = m/rho_eau
# C_f = 0.178549766657
C_f = 0.07

delta_volume_piston = 200 * tick_to_volume

# Piston equilibrium 1040
# Piston initial position : 1150
# Delta = 110
# Volume = tick_to_volume * 110

#################################################
offset = -40 * tick_to_volume

#################################################


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
	if(abs(x[2]-volume_target)<(7.6*tick_to_volume*dt)):
		y[2]= (volume_target-x[2])/dt
	elif(x[2]>volume_target):
		y[2] -= 7.6*tick_to_volume
	elif(x[2]<volume_target):
		y[2] += 7.6*tick_to_volume
	else:
		y[2] = 0
	return y

# 1s / Kv 50 Kf 10
# 1s / Kv 70 Kf 1

delta_t_regulation = 1.0 # sec

def control(d0, d, ddot, V_piston, u):
	global t_old, t
	K_velocity = 100.0
	# if(abs(d0-d)<0.1):
	#  	K = 1000.0
	K_factor = 0.5*(t-t_old)
	t_old = t
	cmd = -K_factor*(-g*((V_piston+offset)*rho_eau/m)-0.5*C_f*ddot*abs(ddot)*rho_eau/m+K_velocity*ddot+(d-d0))
	if(abs(u+cmd)<200):
		# if((np.sign(cmd)==-1 and ddot<-0.06) or (np.sign(cmd)==1 and ddot>0.06)):
		# 	return u
		# else:
		# 	return cmd+u
		return cmd+u
	else:
		return u


# x = np.zeros(3)
x = np.array([0.0, 0.0, 0.0])
# x = np.array([0.0, 0.0, -tick_to_volume * 130])

result_x = []
result_u = []
result_t = []

result_file = []

d0 = 0.75

dt=0.05
t=0
t_old = t-dt
u=0

# time_simulation = 26 # sec
time_simulation = 25*60 # sec

for k in range(0, int(time_simulation/dt)):
	t+=dt

	# if(t<10*60):
	# 	d0=2.0
	# elif(t>10*60 and t <30*60):
	# 	d0=15.0
	# elif(t>30*60 and t <45*60):
	# 	d0=10.0
	# elif(t>45*60 and t <60*60):
	# 	d0=5.0

	if(t<10*60):
		d0=0.5
	elif(t<20*60):
		d0=1.2
	elif(t<40*60):
		d0=0.0

	# if(t>40*60):
	# 	offset = +20 * tick_to_volume

	d = x[0]
	ddot = x[1]
	V_piston = x[2]
	if k % int((delta_t_regulation/dt)) == 0:
		u = control(d0, d, ddot, V_piston, u)
	u_round = round(u)
	dx = f(x, u_round)
	x = x+dx*dt
	if(abs(x[2])>delta_volume_piston):
		x[2] = delta_volume_piston*np.sign(x[2])
	result_x.append(x)
	result_u.append(u_round)
	result_t.append(t)
	if(k%4==0 and k<10000*4 and abs(x[0]-d0)<1.0): # 30 min : 9000*4
		result_file.append([dx[1]+np.random.random_sample()*0.01, x[1]+np.random.random_sample()*0.001, x[2]])

np.savetxt("data_dx.txt", np.transpose(result_file)[0], newline=",")
np.savetxt("data_x1.txt", np.transpose(result_file)[1], newline=",")
np.savetxt("data_x2.txt", np.transpose(result_file)[2], newline=",")

plt.figure(1)
plt.subplot(411)
plt.ylabel('depth')
plt.plot(result_t, np.transpose(result_x)[0])
# ymin, ymax = plt.ylim()  # return the current xlim
# plt.ylim(ymax, ymin)   # set the xlim to xmin, xmax

plt.subplot(412)
plt.ylabel('command (in tick)')
plt.plot(result_t, np.transpose(result_u))

plt.subplot(413)
plt.ylabel('speed')
plt.plot(result_t, np.transpose(result_x)[1])

plt.subplot(414)
plt.ylabel('piston volume (in tick)')
plt.plot(result_t, np.transpose(result_x)[2]/tick_to_volume)
plt.show()


