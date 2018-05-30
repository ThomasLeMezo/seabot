from math import *
import numpy as np
import matplotlib.pyplot as plt

g = 9.81
rho_eau = 1025.0 # kg/m3

R_tube = 0.110/2.0
L_tube = 0.65
V = (R_tube**2)*pi
Cx=10.0
S_Cx=(R_tube**2)*pi

S_piston = ((0.05/2.0)**2)*pi

V_piston_max = 0.1

#m = 12 # kg
m = V*rho_eau

V_estime = V
print("Masse tube equilibre = ", V*rho_eau)
print("Coeff S_Cx*S_Cx", S_Cx*Cx)
print("m = ", m)
print("g*V*rho_eau/m = ", g*V*rho_eau/m)
print("g*(V*rho_eau/m) = ", g*(V*rho_eau/m))

def f(x, u):
	y=np.zeros(3)
	y[0] = x[1]
	y[1] = g - g*((V+x[2])*rho_eau/m) - (0.5*Cx*S_Cx*x[1]*abs(x[1])*rho_eau/m)
	y[2] = u
	return y

def control(d0, d, ddot, V_piston):
	return 0.1*np.sign(-(g-(g*((V_estime+V_piston)*rho_eau/m)+0.5*Cx*S_Cx*ddot*abs(ddot)*rho_eau/m))+2*ddot+(d0-d))

x = np.zeros(3)

result_x = []
result_u = []
d0 = 0.5

dt=0.05
for t in range(0, 100):
	d = x[0]
	ddot = x[1]
	u = control(d0, d, ddot, x[2])
	x = x+f(x, u)*dt
	if(abs(x[2])>V_piston_max):
		x[2] = V_piston_max*np.sign(x[2])
	result_x.append(x)
	result_u.append(u)

plt.figure(1)
plt.subplot(411)
plt.ylabel('depth')
plt.plot(-np.transpose(result_x)[0])

plt.subplot(412)
plt.ylabel('command')
plt.plot(-np.transpose(result_u))

plt.subplot(413)
plt.ylabel('speed')
plt.plot(-np.transpose(result_x)[1])

plt.subplot(414)
plt.ylabel('piston volume')
plt.plot(-np.transpose(result_x)[2])
plt.show()
