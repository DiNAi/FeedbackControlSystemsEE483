from random import random
import control as ctrl
import numpy as np

# Physical parameters
m = 5.0     # mass: Kg
k = 3.0     # spring constant: Kg/s^2
b = 0.5     # damping constant: Kg/s
w = 1       # width of mass
h = 1       # height of mass
l = 3.0     # max length of spring/damper

# Environment parameters
wall_pos = -1
xlim = (-2, 3)
ylim = (-0.2, 2)

# Initial conditions
z0 = 0.0 #l / 2.0
zdot0 = 0.0
Fmax = 10.0

# --------------------------------------------------------------------------- #
# CONTROL PARAMETERS
# --------------------------------------------------------------------------- #

#kp, kd = 3.05, 7.2  # From HW8
Ts = 0.01
tau = 0.05

zeta = 0.707
tr = 2.1
wn = 2.2 / tr

# -- PID -- #
kp = wn**2 *m - k
kd = 2.0*zeta*wn*m - b
ki = 3.7

# -- FULL STATE -- #
A = np.matrix([
	[0.0, 1.0],
	[-k/m, -b/m]
])
B = np.matrix([
	[0.0],
	[1.0/m]
])
C = np.matrix([
	[1.0, 0.0]
])
cl_poles = list(np.roots([1,2*wn*zeta,wn**2]))

K = ctrl.acker(A, B, cl_poles)
kr = -1.0 / np.dot(C, np.dot(np.linalg.inv(A - np.dot(B,K)), B)).item(0)

# -- FULL STATE INTEGRATOR -- #
USE_FULL_STATE_INTEGRATOR = True
if USE_FULL_STATE_INTEGRATOR:
	A1 = np.concatenate((A,-C),0)
	A1 = np.concatenate((A1, np.zeros((A1.shape[0],C.shape[0]))), 1)
	B1 = np.concatenate((B, np.zeros((C.shape[0],B.shape[1]))), 0)

	int_pole = -8.0
	cl_poles.append(int_pole)

	K = ctrl.acker(A1, B1, cl_poles)
	ki = K.take([-1]).item(0)
	K = K.take(range(0, K.shape[1]-1))


# -- OBSERVER -- #
zeta_o = 0.707
wn_o = 4.0*wn
obs_poles = list(np.roots([1,2*zeta_o*wn_o,wn_o**2]))
L = ctrl.acker(A.T, C.T, obs_poles).T


# Add physical error
input_disturbance = 0.25
uncertainty = 0.0#0.2
m += m*uncertainty*(2*random() - 1)
k += k*uncertainty*(2*random() - 1)
b += b*uncertainty*(2*random() - 1)


if __name__ == '__main__':
	print '-- PID control --'
	print '  kp = %s' % kp
	print '  kd = %s' % kd
	print '  ki = %s' % ki
	print '-- Full State Control --'
	print '  K = %s' % K
	print '  kr = %s' % kr
	print '-- Full State Control with Integrator --'
	print '  p_I: %.4f' % cl_poles[2]
	print '  K = %s' % K
	print '  ki = %s' % ki
	print '-- Observer control --'
	print '  poles = {:.4f}, {:.4f}'.format(*[complex(x) for x in obs_poles])
	print '  L = %s' % L
