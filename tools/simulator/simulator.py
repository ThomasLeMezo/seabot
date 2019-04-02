from math import *
import numpy as np
import matplotlib.pyplot as plt


# Vector state x[3]
# x[0] : Velocity
# x[1] : Depth
# x[2] : Piston delta volume

# Parameters
g = 9.81
m = 9.045
rho = 1025.0
d_flange = 0.24
d_piston = 0.05
chi = 7.158e-07 # Compressibility ratio compare to water (m3/m)

Cf = pi*(d_flange/2.0)**2

# Motor parameters
screw_thread = 1.75e-3
tick_per_turn = 48
piston_full_volume = 1.718e-4 # m3
delta_volume_max = 1.718e-4/240.0 # m3/s 

# Regulation
beta = 2./pi*0.03 # Set the limit speed : 5cm/s
root = -1.	 # Set the root of feed-back regulation

l1 = -2.*root
l2 = root**2
A = g*rho/m
B = 0.5*rho*Cf/m

def euler(x, u, dt):
	y=np.array(x)
	y[0] += dt*(-A*(x[2]-chi*x[1])-B*x[0]*abs(x[0]))
	y[1] += dt*x[0]
	y[2] += u
	return y

def control(x, depth_target, dt):
	e = depth_target-x[1]
	y = x[0]-beta*atan(e)
	dx1 = -A*(x[2]-chi*x[1])-B*abs(x[0])*x[0]
	D = 1.+e**2
	dy = dx1 + beta*x[0]/D

	u = (l1*dy+l2*y+beta*(dx1*D+2.*e*x[0]**2)/D**2-2.*B*abs(x[0])*dx1)/A+chi*x[0]
	u_physical = max(min(u, delta_volume_max*dt), -delta_volume_max*dt) ##
	return u_physical

def simulate_passive(x_init, tmax, dt):
	x = np.array(x_init)
	memory = np.append(0., x)
	for t in np.arange(dt, tmax, dt):
		x = euler(x, 0.0, dt)
		memory = np.vstack([memory, np.append(t, x)])
	return memory

def simulate_regulated(x_init, tmax, dt, depth_target):
	x = np.array(x_init)
	memory = np.append(np.append(0., x), 0.)
	for t in np.arange(dt, tmax, dt):
		u = control(x, depth_target, dt)
		x = euler(x, u, dt)
		memory = np.vstack([memory, np.append(np.append(t, x),abs(u*dt)*x[1])])
	return memory

def plot_result(result):
	fig, (ax1, ax2, ax3, ax4) = plt.subplots(4,1, sharex=True)

	ax1.set_ylabel('velocity (m/s)')
	ax1.plot(np.transpose(result)[0], np.transpose(result)[1])

	ax2.set_ylabel('depth (m)')
	ax2.plot(np.transpose(result)[0], np.transpose(result)[2])
	ax2.invert_yaxis()

	ax3.set_ylabel('volume (m3)')
	ax3.plot(np.transpose(result)[0], np.transpose(result)[3])

	ax4.set_ylabel('energy')
	ax4.plot(np.transpose(result)[0], np.transpose(result)[4])

	plt.show()

def plot_velocity_position(result):
	fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)

	ax1.set_ylabel('velocity (m/s)')
	ax1.plot(np.transpose(result)[0], np.transpose(result)[1])

	ax2.set_ylabel('depth (m)')
	ax2.plot(np.transpose(result)[0], np.transpose(result)[2])
	ax2.invert_yaxis()

	plt.show()

def example_passive_more_compressible():
	global chi
	chi = 7.158e-07 # Compressibility (m3/m)
	x_init = np.array([0.0, 0.0, piston_full_volume*0.1])
	result = simulate_passive(x_init, 100., .1)
	plot_velocity_position(result)
	

def example_passive_less_compressible():
	global chi
	chi = -7.158e-07 # Compressibility (m3/m)
	x_init = np.array([0.0, 0.0, piston_full_volume*0.1])
	result = simulate_passive(x_init, 1000., .1)
	plot_velocity_position(result)

def example_regulated_less_compressible():
	global chi
	chi = -7.158e-07 # Compressibility (m3/m)
	depth_target = 5.0
	x_init = np.array([0.0, 0.0, 0.0])
	result = simulate_regulated(x_init, 400., 0.01, depth_target)
	plot_result(result)

def example_regulated_more_compressible():
	global chi
	chi = 7.158e-07 # Compressibility (m3/m)
	depth_target = 1.0
	x_init = np.array([0.0, 0.0, 0.0])
	result = simulate_regulated(x_init, 400., 0.1, depth_target)
	plot_result(result)

if __name__ == "__main__":
	# execute only if run as a script
	# example_passive_more_compressible()
	#example_passive_less_compressible()
	# example_regulated_less_compressible()
	example_regulated_more_compressible()
	




