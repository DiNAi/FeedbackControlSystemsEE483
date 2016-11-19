from random import random
import numpy as np
from scipy.signal import place_poles as place
import control as ctrl

# Physical parameters of the Planar VTOL system
mc = 1       # Mass of center pod: Kg
mr = 0.25    # Mass of right rotor: Kg
ml = 0.25    # Mass of left rotor: Kg
d  = 0.3     # Distance from center of mass to rotor: m
Jc = 0.0042  # Inertia: Kg m^2
u  = 0.1     # Drag: Kg/s
g  = 9.81    # Gravity: m/s^2

rw = 0.13    # Width of center pod: m
rh = 0.1     # Height of center pod: m
rr = 0.038   # Radius of rotors: m
alt= 1.4     # Altitude: m

tw = 0.3     # Width of target: m
th = 0.18    # Height of target: m

# Environmental
xlim = (-3,3)
ylim = (-1,4)

# Initial conditions
z0        = 0.0
h0        = 0.05
theta0    = 0.0
zdot0     = 0.0
hdot0     = 0.0
thetadot0 = 0.0
Fmax      = 20.0
tau_max   = d*Fmax/2.0
Feq = (mc + 2.0*mr)*g

# ------------------------------------------- #
# CONTROL PARAMETERS
# ------------------------------------------- #
Ts = 0.01
tau = 0.05
'''
kp_h = 0.1134
kd_h = 0.5833

kp_th = 0.3721
kd_th = 0.1913

kp_z = -0.00771
kd_z = -0.0328
'''
# ------------------------------- #
# PID
# ------------------------------- #

# Longitudinal
b0 = 1 / (mc + 2.0*mr)
a0 = 1 / (Jc + 2.0*mr*d**2)

tr_h = 0.9
wn_h = 2.2 / tr_h
zeta_h = 0.707
kp_h = wn_h**2 / b0
kd_h = 2.0*zeta_h*wn_h / b0
ki_h = 0.0

# Latteral
tr_th = 0.42
zeta_th = zeta_h
wn_th = 2.2 / tr_th
kp_th = wn_th**2 / a0
kd_th = 2.0*zeta_th*wn_th / a0
ki_th = 0.0

tr_z = tr_th * 2.0
zeta_z = 0.85
wn_z = 2.2 / tr_z
kp_z = -wn_z**2 / (b0*Feq)
kd_z = -(2*zeta_z*wn_z - b0*u) / (b0*Feq)
ki_z = 0.0

# ------------------------------- #
# Full state
# ------------------------------- #
A_lon = np.matrix([
	[0,1],
	[0,0]
])
B_lon = np.matrix([
	[0],
	[1.0/(mc+2.0*mr)]
])
C_lon = np.matrix([
	[1,0]
])

A_lat = np.matrix([
	[0,0,1,0],
	[0,0,0,1],
	[0,-Feq/(mc+2.0*mr),-u/(mc+2.0*mr),0],
	[0,0,0,0]
])
B_lat = np.matrix([
	[0],
	[0],
	[0],
	[1.0/(Jc+2.0*mr*d**2)]
])
C_lat = np.matrix([
	[1,0,0,0],
	[0,1,0,0]
])
Cr_lat = C_lat[0,:]

Cab_lon = ctrl.ctrb(A_lon, B_lon)
cl_poles_lon = list(np.roots([1,2*zeta_h*wn_h,wn_h**2]))
K_lon = ctrl.acker(A_lon, B_lon, cl_poles_lon)
kr_lon = -1.0 / np.dot(C_lon, np.dot(np.linalg.inv(A_lon - np.dot(B_lon,K_lon)), B_lon)).item(0)

Cab_lat = ctrl.ctrb(A_lat, B_lat)
cl_poles_lat = list(np.roots([1,2*zeta_z*wn_z,wn_z**2])) + \
		   list(np.roots([1,2*zeta_th*wn_th,wn_th**2]))
K_lat = ctrl.acker(A_lat, B_lat, cl_poles_lat)
np.dot(C_lat, np.dot(np.linalg.inv(A_lat - np.dot(B_lat,K_lat)), B_lat))
kr_lat = -1.0 / np.dot(C_lat, np.dot(np.linalg.inv(A_lat - np.dot(B_lat,K_lat)), B_lat)).item(0)

# ------------------------------- #
# Full state with integrator
# ------------------------------- #
USE_FULL_STATE_INTEGRATOR = True
if USE_FULL_STATE_INTEGRATOR:
	A1_lon = np.concatenate((A_lon,-C_lon), 0)
	A1_lon = np.concatenate((A1_lon, np.zeros((A1_lon.shape[0],C_lon.shape[0]))), 1)
	B1_lon = np.concatenate(( B_lon, np.zeros((C_lon.shape[0], B_lon.shape[1]))), 0)

	int_pole_lon = -1.0*wn_h
	cl_poles_lon.append(int_pole_lon)

	K1_lon = ctrl.acker(A1_lon, B1_lon, cl_poles_lon)
	ki_lon = K1_lon.take([-1]).item(0)
	K1_lon = K1_lon.take(range(0, K1_lon.shape[1]-1))

	A1_lat = np.concatenate((A_lat,-Cr_lat), 0)
	A1_lat = np.concatenate((A1_lat, np.zeros((A1_lat.shape[0],Cr_lat.shape[0]))), 1)
	B1_lat = np.concatenate(( B_lat, np.zeros((Cr_lat.shape[0], B_lat.shape[1]))), 0)

	int_pole_lat = -1.3*wn_th
	cl_poles_lat.append(int_pole_lat)

	K1_lat = ctrl.acker(A1_lat, B1_lat, cl_poles_lat)
	ki_lat = K1_lat.take([-1]).item(0)
	K1_lat = K1_lat.take(range(0, K1_lat.shape[1]-1))

# ------------------------------- #
# Observer
# ------------------------------- #
zeta_o = 0.707
wn_o_z = 4.0*wn_z
wn_o_th = 4.0*wn_th
wn_o_h = 4.0*wn_h

obs_poles_lon = list(np.roots([1,2*zeta_o*wn_o_h,wn_o_h**2]))
obs_poles_lat = list(np.roots([1,2*zeta_o*wn_o_z,wn_o_z**2])) + list(np.roots([1,2*zeta_o*wn_o_th,wn_o_th**2]))
L_lon = place(A_lon.T, C_lon.T, obs_poles_lon).gain_matrix.T
L_lat = place(A_lat.T, C_lat.T, obs_poles_lat).gain_matrix.T


# Introduce physical error
wind = 0.1
uncertainty = 0.2
mc += mc*uncertainty*(2*random() - 1)
Jc += Jc*uncertainty*(2*random() - 1)
d  +=  d*uncertainty*(2*random() - 1)
u  +=  u*uncertainty*(2*random() - 1)
# Feq = (mc + 2.0*mr)*g

if __name__ == "__main__":
	print '-- PID --'
	print '  Longitudinal:'
	print '    kp_h = %f' % kp_h
	print '    kd_h = %f' % kd_h
	print '    ki_h = %f' % ki_h
	print '  Lateral:'
	print '    kp_th = %f' % kp_th
	print '    kd_th = %f' % kd_th
	print '    ki_th = %f' % ki_th
	print '    kp_z = %f' % kp_z
	print '    kd_z = %f' % kd_z
	print '    ki_z = %f' % ki_z
	print '-- Full state --'
	print '  K_lon = %s' % K_lon
	print '  kr_lon = %f' % kr_lon
	print '  K_lat = %s' % K_lat
	print '  kr_lat = %f' % kr_lat
	print '-- Full state with integrator --'
	print '  K1_lon = %s' % K1_lon
	print '  ki_lon = %f' % ki_lon
	print '  K1_lat = %s' % K1_lat
	print '  ki_lat = %f' % ki_lat
	print '-- Observer control --'
	print '  h poles = {:.3f}, {:.3f}'.format(*obs_poles_lon)
	print '  L_lon = \n%s' % L_lon
	print '  z poles = {:.3f}, {:.3f}'.format(*obs_poles_lat[:2])
	print '  theta poles = {:.3f}, {:.3f}'.format(*obs_poles_lat[2:])
	print '  L_lat = \n%s' % L_lat

