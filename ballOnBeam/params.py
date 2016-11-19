from random import random
import numpy as np
from scipy.signal import place_poles as place
import control as ctrl

# Parameters for ball on beam system
# Physical parameters
m1    = 0.35 # Mass of the ball: Kg
m2    = 2.0  # Mass of the beam: Kg
l     = 0.5  # Length of the beam: m
g     = 9.8  # Force of gravity: Kg/m^2
r     = 0.05 # Radius of the ball: m
w     = l / 35.0 # Width of the beam: m

# Environment parameters
xlim = (-0.1, 0.6)
ylim = (-0.6, 0.6)

# Initial conditions
z0 = l/2.0
theta0 = 0.0
zdot0 = 0.0
thetadot0 = 0.0

Feq = m1*g/l*z0 + m2*g/2.0
Fmax = 15.0


# --------------------------------------------- #
# CONTROL PARAMETERS
# --------------------------------------------- #
Ts = 0.01
tau = 0.05

tr_th = 0.48
wn_th = 2.2 / tr_th
zeta_th = 0.707

tr_z = 1.5*tr_th
wn_z = 2.2 / tr_z
zeta_z = 0.8

# ------------------------------ #
# pid
# ------------------------------ #
# th_kp = 1.825
# th_kd = 1.173
# z_kp = -0.00494
# z_kd = -0.03174

# theta loop
b0 = l / (1.0/3.0*m2*l**2 + 1.0/4.0*m1*l**2)
kp_th = wn_th**2 / b0
kd_th = 2.0*zeta_th*wn_th / b0
ki_th = -0.0
# z loop
zeta_z = 0.75
kp_z = -wn_z**2 / g
kd_z = -2.0*zeta_z*wn_z / g
ki_z = -0.0

# ------------------------------ #
# full state
# ------------------------------ #

d_0 = (1.0/3*m2*l**2 + m1*z0**2)
a_0 = (-m1*g) / d_0
b_0 = l / d_0
A = np.matrix([
	[0,0,1,0],
	[0,0,0,1],
	[0,-g,0,0],
	[a_0,0,0,0]
])
B = np.matrix([
	[0],
	[0],
	[0],
	[b_0]
])
C = np.matrix([
	[1,0,0,0],
	[0,1,0,0]
])
Cr = C[0,:]

Cab = ctrl.ctrb(A, B)
cl_poles = list(np.roots([1,2*zeta_z*wn_z,wn_z**2])) + list(np.roots([1,2*zeta_th*wn_th,wn_th**2]))

K = ctrl.acker(A, B, cl_poles)
kr = -1.0 / np.dot(C, np.dot(np.linalg.inv(A - np.dot(B,K)), B)).item(0)

# ------------------------------ #
# full state with integrator
# ------------------------------ #

USE_FULL_STATE_INTEGRATOR = True
if USE_FULL_STATE_INTEGRATOR:
	A1 = np.concatenate((A,-Cr), 0)
	A1 = np.concatenate((A1, np.zeros((A1.shape[0],Cr.shape[0]))), 1)
	B1 = np.concatenate(( B, np.zeros((Cr.shape[0], B.shape[1]))), 0)

	int_pole = -4.0*wn_th
	cl_poles.append(int_pole)

	K1 = ctrl.acker(A1, B1, cl_poles)
	ki = K1.take([-1]).item(0)
	K1 = K1.take(range(0, K1.shape[1]-1))


# -- OBSERVER -- #
zeta_o = 0.707
wn_o_z = 4.0*wn_z
wn_o_th = 4.0*wn_th
obs_poles = list(np.roots([1,2*zeta_o*wn_o_z,wn_o_z**2])) + list(np.roots([1,2*zeta_o*wn_o_th,wn_o_th**2]))
L = place(A.T, C.T, obs_poles).gain_matrix.T

	
# Add physical parameter error
input_disturbance = 0.5
uncertainty = 0.0
m1 += m1*uncertainty*(2*random() - 1)
m2 += m2*uncertainty*(2*random() - 1)
l += l*uncertainty*(2*random() - 1)


if __name__ == '__main__':
	print '-- PID --'
	print '  Z:' 
	print '    kp = %f' % kp_z
	print '    kd = %f' % kd_z
	print '    ki = %f' % ki_z
	print '  Theta:' 
	print '    kp = %f' % kp_th
	print '    kd = %f' % kd_th
	print '    ki = %f' % ki_th
	print '-- Full state --'
	print '  K = %s' % K
	print '  kr = %f' % kr
	print '  Rank of controllability matrix: %s' % np.linalg.matrix_rank(Cab)
	print '-- Full state with integrator --'
	print '  K1 = %s' % K1
	print '  ki = %f' % ki
	print '-- Observer control --'
	print '  z poles = {:.3f}, {:.3f}'.format(*[complex(x) for x in obs_poles[:2]])
	print '  theta poles = {:.3f}, {:.3f}'.format(*[complex(x) for x in obs_poles[2:]])
	print '  L = %s' % L