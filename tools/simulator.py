from math import *
import numpy as np
import matplotlib.pyplot as plt

tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*np.pi

g = 9.81
rho_eau = 1025.0 # kg/m3

R_tube = 0.110/2.0
L_tube = 0.65
V = (R_tube**2)*pi * L_tube
Cx=1.0
S_Cx=(R_tube**2)*pi

S_piston = ((0.05/2.0)**2)*pi

V_piston_max = 200 * tick_to_volume

#################################################
offset = -(40-61) * tick_to_volume

#################################################

m = V*rho_eau

V_estime = V
print("Masse tube equilibre = ", V*rho_eau)
print("Coeff S_Cx*Cx", S_Cx*Cx)
print("m = ", m)
print("g*V*rho_eau/m = ", g*V*rho_eau/m)
print("g*(V*rho_eau/m) = ", g*(V*rho_eau/m))
print("V = ", V)
print("offset = ", offset)
print("tick_to_volume = ", tick_to_volume)

def f(x, u):
	y=np.zeros(3)
	y[0] = x[1]
	y[1] = g - g*((V+x[2]+offset)*rho_eau/m) - (0.5*Cx*S_Cx*x[1]*abs(x[1])*rho_eau/m)
	y[2] = u
	return y

def control(d0, d, ddot, V_piston):
	K = 100.0
	if(abs(d0-d)<0.2):# and abs(ddot) < 0.001):
	 	K = 1000.0
	return 0.000001*(-(g-g*((V_estime+V_piston)*rho_eau/m)-0.5*Cx*S_Cx*ddot*abs(ddot)*rho_eau/m)+K*ddot+(d-d0))

x = np.zeros(3)

result_x = []
result_u = []
result_t = []

result_file = []

d0 = 2.0

dt=0.05
t=0
u=0
delta_t_regulation = 30 # sec
time_simulation = 60*60 # sec

for k in range(0, int(time_simulation/dt)):
	t+=dt
	d = x[0]
	ddot = x[1]
	V_piston = x[2]
	if k % int((delta_t_regulation/dt)) == 0:
		u = control(d0, d, ddot, V_piston)
	else:
		u = 0
	cmd = round(u/tick_to_volume)
	u = round(u/tick_to_volume)*tick_to_volume
	dx = f(x, u)
	x = x+dx*dt
	if(abs(x[2])>V_piston_max):
		x[2] = V_piston_max*np.sign(x[2])
	result_x.append(x)
	result_u.append(cmd)
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
ymin, ymax = plt.ylim()  # return the current xlim
plt.ylim(ymax, ymin)   # set the xlim to xmin, xmax

plt.subplot(412)
plt.ylabel('command')
plt.plot(result_t, np.transpose(result_u))

plt.subplot(413)
plt.ylabel('speed')
plt.plot(result_t, np.transpose(result_x)[1])

plt.subplot(414)
plt.ylabel('piston tick')
plt.plot(result_t, np.transpose(result_x)[2]/tick_to_volume)
plt.show()


