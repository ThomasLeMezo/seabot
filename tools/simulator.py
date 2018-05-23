from math import *
import numpy as np

g = 9.81
rho_eau = 1025 # kg/m3
m = 5 # kg
R_tube = 0.12/2.0
L_tube = 0.65
V = (R_tube**2)*pi
Cx=1
S_Cx=(R_tube**2)*pi

S_piston = ((0.05/2.0)**2)*pi

def f(x, u):
	y=x
	y[0] = x[0]
	y[1] = g - (rho_eau*g*(V+y[2]) - 0.5*rho_eau*Cx*S_Cx*y[1]*abs(y[1]))/m
	y[2] = u
	return y

x = np.zeros(3)

dt=0.1
for t in range(0, 10000):
	x = x+f(x, 0)*dt

print(x)

