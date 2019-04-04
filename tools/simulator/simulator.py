from math import *
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv, det, norm, eig
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round

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

tick_to_volume = (screw_thread/tick_per_turn)*((d_piston/2.0)**2)*np.pi

tick_offset = 250.0

# Regulation
beta = 2./pi*0.03 # Set the limit speed : 5cm/s
root = -1.	 # Set the root of feed-back regulation

l1 = -2.*root
l2 = root**2
A_coeff = g*rho/m
B_coeff = 0.5*rho*Cf/m

def f(x, u, gamma):
	y = np.array(x)
	y[0] = -A_coeff*(u+x[2]-chi*x[1])-B_coeff*x[0]*abs(x[0])
	y[1] = x[0]
	y[2] = 0.
	return y

def kalman_predict(xup,Gup,u,gamma_alpha,A, dt):
    gamma1 = A @ Gup @ A.T + gamma_alpha
    x1 = xup + f(xup, u, gamma1)*dt
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
	y[0] += dt*(-A_coeff*(x[2]-chi*x[1])-B_coeff*x[0]*abs(x[0]))
	y[1] += dt*x[0]
	y[2] += u
	return y

def control(x, depth_target, dt):
	e = depth_target-x[1]
	y = x[0]-beta*atan(e)
	dx1 = -A_coeff*(x[2]-chi*x[1])-B_coeff*abs(x[0])*x[0]
	D = 1.+e**2
	dy = dx1 + beta*x[0]/D

	u = (l1*dy+l2*y+beta*(dx1*D+2.*e*x[0]**2)/D**2-2.*B_coeff*abs(x[0])*dx1)/A_coeff+chi*x[0]
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

	volume_offset = tick_offset*tick_to_volume

	x_hat = np.array([0.0, 0.0, 0.0])
	gamma = np.array([[(1e-1)**2, 0, 0],
					  [0, (1e-2)**2, 0],
					  [0, 0, (1e-5)**2]])
	gamma_alpha = np.array([[(1e-3)**2, 0, 0],
					  		[0, (1e-4)**2, 0],
					  		[0, 0, (1e-8)**2]])
	gamma_beta = np.array([(1e-3)**2]) # measure

	A = np.array([[0., chi*A_coeff, -A_coeff],
					  [1, 0, 0],
					  [0, 0, 0.]])
	C = np.array([[0, 1, 0]])

	memory = np.append(np.append(0., x), 0.)
	memory_kalman = np.append(0., (x_hat))
	memory_kalman_cov = np.append(0., np.array([gamma[0][0], gamma[1][1], gamma[2][2]]))
	for t in np.arange(dt, tmax, dt):
		u = control(x, depth_target, dt)
		x = euler(x, u, dt)
		memory = np.vstack([memory, np.append(np.append(t, x),abs(u*dt)*x[1])])

		# Kalman
		cmd = x[2]-volume_offset
		y = x[1] #+ np.random.normal(1.0, (1e-3)**2)
		A[0][0] = -2.*B_coeff*x_hat[0] # x_hat[0]
		(x_hat,gamma) = kalman(x_hat,gamma,cmd,y,gamma_alpha,gamma_beta,A,C, dt)		
		memory_kalman = np.vstack([memory_kalman, np.append(t, (x_hat))])
		memory_kalman_cov = np.vstack([memory_kalman_cov, np.append(t, (np.array([gamma[0][0], gamma[1][1], gamma[2][2]])))])

	print("volume_offset = ", volume_offset)
	print("x_hat = ", x_hat)
	print("x     = ", x)
	print("Cov = ", gamma)
	return (memory, memory_kalman, memory_kalman_cov)

def plot_result(result):
	fig, (ax1, ax2, axes[0,2], ax4) = plt.subplots(4,1, sharex=True)

	ax1.set_ylabel('x1 (velocity [m/s])')
	ax1.plot(np.transpose(result)[0], np.transpose(result)[1])

	ax2.set_ylabel('x2 (depth [m])')
	ax2.plot(np.transpose(result)[0], np.transpose(result)[2])
	ax2.invert_yaxis()

	ax3.set_ylabel('x3 (volume [m3])')
	ax3.plot(np.transpose(result)[0], np.transpose(result)[3])

	ax4.set_ylabel('energy')
	ax4.plot(np.transpose(result)[0], np.transpose(result)[4])

	plt.show()

def plot_result_kalman(result_kalman, result_euler, result_cov):
	fig, axes = plt.subplots(3,2)

	axes[0,0].set_ylabel('x1 (velocity [m/s])')
	axes[0,0].plot(np.transpose(result_kalman)[0], np.transpose(result_kalman)[1])
	axes[0,0].plot(np.transpose(result_kalman)[0], np.transpose(result_euler)[1])

	axes[1,0].set_ylabel('x2 (depth [m])')
	axes[1,0].plot(np.transpose(result_kalman)[0], np.transpose(result_kalman)[2])
	axes[1,0].plot(np.transpose(result_kalman)[0], np.transpose(result_euler)[2])
	axes[1,0].invert_yaxis()

	axes[2,0].set_ylabel('x4 (volume offset [m3])')
	axes[2,0].plot(np.transpose(result_kalman)[0], np.transpose(result_kalman)[3])

	axes[0,1].set_ylabel('cov x1 (velocity)')
	axes[0,1].plot(np.transpose(result_cov)[0], np.transpose(result_cov)[1])

	axes[1,1].set_ylabel('cov x2 (depth)')
	axes[1,1].plot(np.transpose(result_cov)[0], np.transpose(result_cov)[2])

	axes[2,1].set_ylabel('cov x4 (offset)')
	axes[2,1].plot(np.transpose(result_cov)[0], np.transpose(result_cov)[3])

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
	# chi = 7.158e-07 # Compressibility (m3/m)
	chi = 0.0
	depth_target = 5.0
	x_init = np.array([0.0, 0.0, 0.0])
	(memory, memory_kalman, memory_kalman_cov) = simulate_regulated(x_init, 400., 0.1, depth_target)
	# plot_result(memory)
	plot_result_kalman(memory_kalman, memory, memory_kalman_cov)

if __name__ == "__main__":
	# execute only if run as a script
	# example_passive_more_compressible()
	#example_passive_less_compressible()
	# example_regulated_less_compressible()
	example_regulated_more_compressible()
	




