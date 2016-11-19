import numpy as np
import params as P


class RotorVtolDynamics(object):
	
	def __init__(self, controller):
		self.state = np.matrix([
			[P.z0],
			[P.h0],
			[P.theta0],
			[P.zdot0],
			[P.hdot0],
			[P.thetadot0]
		])
		self.controller = controller
		self.mass_tot = (P.mc + 2*P.mr)


	def propogateDynamics(self, ref_input):
		'''ref_input is the z and h reference commands'''

		u = self.controller.getForces(ref_input, self.state)

		# RK4 integration
		k1 = self.Derivatives(self.state, u)
		k2 = self.Derivatives(self.state + P.Ts/2.0*k1, u)
		k3 = self.Derivatives(self.state + P.Ts/2.0*k2, u)
		k4 = self.Derivatives(self.state + P.Ts*k3, u)
		self.state += P.Ts/6.0 * (k1 + 2*k2 + 2*k3 + k4)


	def Derivatives(self, state, u):
		'''Return the derivatives of the state'''

		z = state.item(0)
		h = state.item(1)
		theta = state.item(2)
		zdot = state.item(3)
		hdot = state.item(4)
		thetadot = state.item(5)
		mc, mr, Jc, g, d = P.mc, P.mr, P.Jc, P.g, P.d

		F,tau = u
#		fl = 1.0/(2.0)*(F-tau/P.d)
#		fr = 1.0/(2.0)*(F+tau/P.d)

		# horizontal movement dynamics
		zddot = (-P.u*zdot - F*np.sin(theta) + P.wind) / self.mass_tot
#		zddot = -(fr+fl)*np.sin(theta)/(P.mc+2*P.mr)
		# vertical movement dynamics
		hddot = (F*np.cos(theta) / self.mass_tot) - g
#		hddot = (-(P.mc+2*P.mr)*P.g + (fr+fl)*np.cos(theta))/(P.mc+2*P.mr)
		# rotational movement dynamics
		thetaddot = tau / (Jc + 2*mr*(d**2))
#		thetaddot = P.d*(fr-fl)/(P.Jc+2*P.mr*P.d**2)

#		print zddot, hddot, thetaddot
		
		return np.matrix([
			[zdot],
			[hdot],
			[thetadot], 
			[zddot],
			[hddot],
			[thetaddot]
		])


	def Outputs(self):
		return self.state[:3].T.tolist()[0]


	def States(self):
		return self.state.T.tolist()[0]
