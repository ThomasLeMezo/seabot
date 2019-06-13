from math import *
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv, det, norm, eig
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
					exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
					arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round

from vibes import vibes
# Vector state x[3]
# x[0] : Velocity
# x[1] : Depth
# x[2] : Piston delta volume

# Parameters
g = 9.81
m = 9.045*2.0
rho = 1025.0
d_flange = 0.24
d_piston = 0.05

Cf = 1.0*pi*(d_flange/2.0)**2

# Motor parameters
screw_thread = 1.75e-3
tick_per_turn = 48
piston_full_volume = 1.718e-4 # m3

tick_to_volume = (screw_thread/tick_per_turn)*((d_piston/2.0)**2)*np.pi
delta_volume_max = tick_to_volume*500. # m3/s 
velocity_volume_max = 30.*tick_to_volume

tick_offset = 50.0
chi = 30.0*tick_to_volume # Compressibility ratio compare to water (m3/m)

# Regulation
beta = (2./pi)*0.0358 # Set the limit speed : [ex: 0.03 m/s]
root = -0.3	 # Set the root of feed-back regulation

l1 = -2.*root
l2 = root**2
A_coeff = g*rho/m
B_coeff = 0.5*rho*Cf/m

vertical_velocity = 0.0

def f(x, u, gamma_alpha):
	y = np.array(x)
	y[0] = -A_coeff*(u+x[2]-x[3]*x[1])-B_coeff*x[0]*abs(x[0])
	y[1] = x[0]
	y[2] = 0.0
	y[3] = 0.0
	return y

def kalman_predict(xup,Gup,u,gamma_alpha,A, dt):
	gamma1 = A @ Gup @ A.T + gamma_alpha
	x1 = xup + f(xup, u, gamma_alpha)*dt
	return(x1,gamma1)

def kalman_correc(x0,gamma0,y,gamma_beta,C):
	S = C @ gamma0 @ C.T + gamma_beta
	K = gamma0 @ C.T @ inv(S)
	ytilde = y - C @ x0
	Gup = (eye(len(x0))-K @ C) @ gamma0
	xup = x0 + K@ytilde
	return(xup,Gup) 
	
def kalman(x0,gamma0,u,y,gamma_alpha,gamma_beta,A,C, dt):
	xup,Gup = kalman_correc(x0,gamma0,y,gamma_beta,C)
	x1,gamma1=kalman_predict(xup,Gup,u,gamma_alpha,A, dt)
	return(x1,gamma1)  

def euler(x, u, dt):
	y=np.array(x)
	y[0] += dt*(-A_coeff*(x[2]-chi*x[1])-B_coeff*x[0]*abs(x[0])+B_coeff*((vertical_velocity-x[0])*(abs(vertical_velocity-x[0]))))
	y[1] += dt*(x[0])
	y[2] += dt*u
	y[2] = max(min(y[2], delta_volume_max), -delta_volume_max)
	return y

def control(x, depth_target, dt, chi_kalman):
	e = depth_target-x[1]
	y = x[0]-beta*atan(e)
	dx1 = -A_coeff*(x[2]-chi_kalman*x[1])-B_coeff*abs(x[0])*x[0]
	D = 1.+e**2
	dy = dx1 + beta*x[0]/D

	u = (l1*dy+l2*y+beta*(dx1*D+2.*e*(x[0]**2))/(D**2)-2.*B_coeff*abs(x[0])*dx1)/A_coeff+chi_kalman*x[0]
	u_physical = max(min(u, velocity_volume_max*dt), -velocity_volume_max*dt)
	u_physical = round(u_physical/tick_to_volume)*tick_to_volume
	return u_physical

def simulate_passive(x_init, tmax, dt, z_limit):
	x = np.array(x_init)
	memory = np.append(0., x)
	for t in np.arange(dt, tmax, dt):
		x = euler(x, 0.0, dt)
		memory = np.vstack([memory, np.append(t, x)])
		if(abs(x[1])>z_limit):
			return memory
	return memory

def simulate_regulated(x_init, tmax, dt, depth_target):
	x = np.array(x_init)

	volume_offset = tick_offset*tick_to_volume

	x_hat = np.array([0.0, 0.0, 0.0, 0.0])
	gamma = np.array([[(1e-1)**2,	0.0, 		0.0, 		0.0],
					  [0.0,			(1e-3)**2, 	0.0, 		0.0],
					  [0.0, 		0.0, 		(tick_to_volume*250.)**2, 	0.0	],
					  [0.0, 		0., 		0.0, 		(tick_to_volume*250.)**2]])

	gamma_alpha = np.array([[(1e-4)**2, 0, 0, 0],
							[0, (1e-5)**2, 0, 0],
							[0, 0, (1e-8)**2, 0],
							[0, 0, 	0, 		(1e-8)**2]])

	gamma_beta = np.array([(1e-4)**2]) # measure

	A = np.array([	[0., 0., -A_coeff, 0.],
					[1, 0., 0, 0],
					[0, 0, 0., 0],
					[0, 0, 0, 0.]])
	C = np.array([[0, 1, 0, 0.]])

	memory = np.append(np.append(0., x), np.array([0., 0.]))
	memory_kalman = np.append(0., (x_hat))
	memory_kalman_cov = np.append(0., np.array([gamma[0][0], gamma[1][1], gamma[2][2], gamma[3][3]]))
	for t in np.arange(dt, tmax, dt):
		
		# Kalman
		cmd = np.array(x[2]-volume_offset) + tick_to_volume * np.random.normal(0.0, (1.)**2)
		y = x[1] + np.random.normal(0.0, 1e-3)
		A[0][0] = -2.*B_coeff*abs(x_hat[0]) # x_hat[0]
		A[0][1] = A_coeff*x_hat[3]
		A[0][3] = A_coeff*x_hat[1]

		(x_hat,gamma) = kalman(x_hat,gamma,cmd,y,gamma_alpha,gamma_beta,A*dt+np.eye(4),C, dt)		
		memory_kalman = np.vstack([memory_kalman, np.append(t, (x_hat))])
		memory_kalman_cov = np.vstack([memory_kalman_cov, np.append(t, (np.array([gamma[0][0], gamma[1][1], gamma[2][2], gamma[3][3]])))])

		x_control = np.array([0.0, 0.0, 0.0]) # v,z,V
		x_control[0] = x_hat[0]
		x_control[1] = x_hat[1]
		x_control[2] = (x[2]-volume_offset)+x_hat[2]
		chi_kalman = x_hat[3]

		if(t>=960. and t<1860.*2.):
			depth_target = 1.75
		elif(t>=1860.*2.):
			depth_target = 1.

		u = control(x_control, depth_target, dt, chi_kalman)
		memory = np.vstack([memory, np.append(np.append(t, x),np.array([abs(u*dt)*x[1], depth_target]))])

		x = euler(x, u, dt)

	print("volume_offset = ", volume_offset)
	print("x_hat = ", x_hat)
	print("x     = ", x)
	print("Cov = ", gamma)
	return (memory, memory_kalman, memory_kalman_cov)

def plot_result(result):
	fig, (ax1, ax2, ax3, ax4) = plt.subplots(4,1, sharex=True)

	ax1.set_ylabel('x1 (velocity [m/s])')
	ax1.plot(np.transpose(result)[0], np.transpose(result)[1])

	ax2.set_ylabel('x2 (depth [m])')
	ax2.plot(np.transpose(result)[0], np.transpose(result)[2])
	# ax2.invert_yaxis()

	ax3.set_ylabel('x3 (volume [m3])')
	ax3.plot(np.transpose(result)[0], np.transpose(result)[3])

	ax4.set_ylabel('energy')
	ax4.plot(np.transpose(result)[0], np.transpose(result)[4])

	plt.show()

def plot_result_kalman(result_kalman, result_euler, result_cov):
	fig, axes = plt.subplots(4,2)

	axes[0,0].set_ylabel('x1 (velocity [m/s])')
	axes[0,0].plot(np.transpose(result_kalman)[0], np.transpose(result_kalman)[1])
	axes[0,0].plot(np.transpose(result_kalman)[0], np.transpose(result_euler)[1])

	axes[1,0].set_ylabel('x2 (depth [m])')
	axes[1,0].plot(np.transpose(result_kalman)[0], np.transpose(result_kalman)[2])
	axes[1,0].plot(np.transpose(result_kalman)[0], np.transpose(result_euler)[2])
	axes[1,0].plot(np.transpose(result_kalman)[0], np.transpose(result_euler)[5])
	# axes[1,0].invert_yaxis()

	axes[2,0].set_ylabel('x4 (volume offset [m3])')
	axes[2,0].plot(np.transpose(result_kalman)[0], np.transpose(result_kalman)[3]/tick_to_volume)

	axes[3,0].set_ylabel('x5 (chi [m3])')
	axes[3,0].plot(np.transpose(result_kalman)[0], np.transpose(result_kalman)[4]/tick_to_volume)

	axes[0,1].set_ylabel('cov x1 (velocity)')
	axes[0,1].plot(np.transpose(result_cov)[0], np.transpose(result_cov)[1])

	axes[1,1].set_ylabel('cov x2 (depth)')
	axes[1,1].plot(np.transpose(result_cov)[0], np.transpose(result_cov)[2])

	axes[2,1].set_ylabel('cov x4 (offset)')
	axes[2,1].plot(np.transpose(result_cov)[0], np.transpose(result_cov)[3])

	axes[3,1].set_ylabel('cov x5 (chi)')
	axes[3,1].plot(np.transpose(result_cov)[0], np.transpose(result_cov)[4])

	plt.show()

def plot_velocity_position(result):
	
	# vibes.drawLine((np.transpose(np.transpose(result)[0:2])).tolist())
	vibes.drawLine(result[:,[0,2]].tolist())
	# fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)

	# ax1.set_ylabel('velocity (m/s)')
	# ax1.plot(np.transpose(result)[0], np.transpose(result)[1])

	# ax2.set_ylabel('depth (m)')
	# ax2.plot(np.transpose(result)[0], np.transpose(result)[2])
	# ax2.invert_yaxis()

	# plt.show()

def example_passive_more_compressible():
	global chi
	chi = 2.14e-6 # Compressibility (m3/m)
	x_init = np.array([0.0, 0.1, piston_full_volume*0.])
	result = simulate_passive(x_init, 1000., .1, 2)
	plot_velocity_position(result)
	

def example_passive_less_compressible():
	global chi
	chi = -2.14e-6 # Compressibility (m3/m)
	x_init = np.array([0.0, 0.1, piston_full_volume*0.])
	result = simulate_passive(x_init, 1000., .1, 2)
	plot_velocity_position(result)

def example_regulated_less_compressible():
	global chi
	chi = -7.158e-07 # Compressibility (m3/m)
	depth_target = 5.0
	x_init = np.array([0.0, 0.0, 0.0])
	result = simulate_regulated(x_init, 400., 0.1, depth_target)
	plot_result(result)

def example_regulated_more_compressible():
	global chi
	# chi = 7.158e-07 # Compressibility (m3/m)
	# chi = 0.0
	depth_target = 1.0
	x_init = np.array([0.0, 0.0, 0.0])
	(memory, memory_kalman, memory_kalman_cov) = simulate_regulated(x_init, 1860., 1., depth_target)
	plot_result(memory)
	plot_result_kalman(memory_kalman, memory, memory_kalman_cov)

def example_oscillation_command():
	global chi
	chi = -30.*tick_to_volume
	x = np.array([0.0, 0.0, -100.*tick_to_volume/2.0])
	dt = 0.1
	tmax = 1000.
	memory = np.append(np.append(0., x), 0.)
	k=0
	V=x[2]
	for t in np.arange(dt, tmax, dt):
		if(k>(1./0.1)/1.):
			k=0
			V=-V
		else:
			k+=1
		x[2] = V
		x = euler(x, 0., dt)
		memory = np.vstack([memory, np.append(np.append(t, x),abs(0.*dt)*x[1])])
	plot_result(memory)

if __name__ == "__main__":
	# execute only if run as a script
	
	
	# example_regulated_less_compressible()
	example_regulated_more_compressible()

	# vibes.beginDrawing()
	# vibes.newFigure("Float_position")
	# vibes.setFigureProperties( { "x": 100,"y": 100,"width": 1000,"height": 500} )
	# vibes.axisLimits(0, 1000, -0.2, 2)
	# vibes.drawBox(0, 1000, -0.2, 2, "white[white]")
	# example_passive_more_compressible()
	# example_passive_less_compressible()
	# vibes.saveImage("/home/lemezoth/workspaceQT/tikz-adapter/tikz/figs/svg/compressibility.svg")
	# vibes.endDrawing()
	
	# example_oscillation_command()




