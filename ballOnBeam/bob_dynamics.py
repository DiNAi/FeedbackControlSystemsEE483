import numpy as np
import params as P


class BallOnBeamDynamics(object):
	
	def __init__(self, controller):
		self.state = np.matrix([
			[P.z0],
			[P.theta0],
			[P.zdot0],
			[P.thetadot0]
		])
		self.controller = controller


	def propogateDynamics(self, ref_input):
		'''ref_input: desired z position of ball'''
		u = ref_input

		# Calculate the input force as determined by the controller
		F = self.controller.getForces(ref_input, self.state)[0] + P.input_disturbance

		# RK4 integration
		k1 = self.Derivatives(self.state, u, F)
		k2 = self.Derivatives(self.state + P.Ts/2.0*k1, u, F)
		k3 = self.Derivatives(self.state + P.Ts/2.0*k2, u, F)
		k4 = self.Derivatives(self.state + P.Ts*k3, u, F)
		self.state += P.Ts/6.0 * (k1 + 2*k2 + 2*k3 + k4)


	def Derivatives(self, state, ref_input, F):
		'''Return the derivatives of the state'''

		z = state.item(0)
		theta = state.item(1)
		zdot = state.item(2)
		thetadot = state.item(3)

		#F = self.controller.getForces(ref_input, self.state)[0]

		# Ball dynamics
		zddot = z*thetadot**2 - P.g*np.sin(theta)
		# Ramp dynamics
		m1, m2, l, g = P.m1, P.m2, P.l, P.g
		cos = np.cos(theta)
		thetaddot = ((F*l*cos - 2.0*m1*z*zdot*thetadot - m1*g*z*cos - m2*g*l*cos/2.0) / 
								(m2*(l**2)/3.0 + m1*(z**2)))
		
		return np.matrix([
			[zdot],
			[thetadot], 
			[zddot],
			[thetaddot]
		])


	def Outputs(self):
		return self.state[:2].T.tolist()[0]


	def States(self):
		return self.state.T.tolist()[0]
