import numpy as np
import params as P
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from controller import Controller


class MassSpringDamperDynamics(object):
	
	def __init__(self, controller):
		self.state = np.matrix([
			[float(P.z0)],
			[float(P.zdot0)]
		])

		self.ctrl = controller


	def propogateDynamics(self, ref_input):
		'''ref_input is the reference position'''

		u = self.ctrl.getForces(ref_input[0], self.state.item(0)) + P.input_disturbance
		# RK4 integration
		k1 = self.Derivatives(self.state, u)
		k2 = self.Derivatives(self.state + P.Ts/2.0*k1, u)
		k3 = self.Derivatives(self.state + P.Ts/2.0*k2, u)
		k4 = self.Derivatives(self.state + P.Ts*k3, u)
		der = P.Ts/6.0 * (k1 + 2*k2 + 2*k3 + k4)
		self.state += der


	def Derivatives(self, state, u):
		'''Return the derivatives of the state'''

		z = state.item(0)
		zdot = state.item(1)
		F = u

		# m*zddot + b*zdot + k*z = F
		# zddot = (F - b*zdot - k*z) / m
		zddot = (F - P.b*zdot - P.k*z) / P.m
		return np.matrix([[zdot], [zddot]])


	def Outputs(self):
		return self.state.item(0)


	def States(self):
		return self.state.T.tolist()[0]
