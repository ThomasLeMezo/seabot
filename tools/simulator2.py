from math import *
import numpy as np
import matplotlib.pyplot as plt

g = 9.81
rho = 1025.0 # kg/m3
m=8.3
Cf = np.pi*(0.12)**2

A=g*rho/m
B=0.5*rho*Cf/m

tick_to_volume = (1.75e-3/48.0)*((0.05/2.0)**2)*np.pi

x1=0.
x2=0.
x3=0.
dt=0.01
set_point=5.

def euler(u):
	global x1, x2, x3
	x1= x1+(-A*x3-B*x1**2*np.sign(x1))*dt
	x2= x2+x1*dt
	x3= x3+u*dt

def control():
	x3_estim = x3 - 4.*tick_to_volume

	beta = -0.02*np.pi/2.0
	l = 100.

	e=set_point-x2
	dx1 = -A*x3_estim-B*(x1**2)*np.sign(x1)

	y = x1 + beta * atan(e)
	dy = dx1-beta*x1/(1.+e**2)

	u = (1./A)*(-2.*B*x1*dx1*np.sign(x1) -beta*(2.*(x1**2)*e + dx1*(1.+e**2))/((1.+e**2)**2) +l*dy+y)
	ddy = -A*u-2.*B*x1*dx1*np.sign(x1)-beta*(2*x1**2*e+dx1*(1.+e**2))/((1+e**2)**2)

	eq = y+l*dy+ddy

	y_log.append(y)
	dy_log.append(dy)
	ddy_log.append(ddy)
	eq_log.append(y+l*dy+ddy)

	return u

x1_log = [x1]
x2_log = [x2]
x3_log = [x3]
y_log = []
dy_log = []
ddy_log = []
eq_log = []

for i in range(100000):
	# euler()
	u = control()
	euler(u)
	
	x1_log.append(x1)
	x2_log.append(x2)
	x3_log.append(x3)

fig, (ax1, ax2, ax3) = plt.subplots(3,1, sharex=True)
ax1.set_ylabel('x1')
ax1.plot(x1_log)
ax2.set_ylabel('x2')
ax2.plot(x2_log)
ax3.set_ylabel('x3')
ax3.plot(x3_log)

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4,1, sharex=True)
ax1.set_ylabel('y')
ax1.plot(y_log)
ax2.set_ylabel('dy')
ax2.plot(dy_log)
ax3.set_ylabel('ddy')
ax3.plot(ddy_log)
ax4.set_ylabel('eq')
ax4.plot(eq_log)

plt.show()
