import numpy as np
import control as ctrl
import params as P
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from controller import *


class MassSpringDamperControllerPID(Controller):

	def __init__(self):

		super(MassSpringDamperControllerPID, self).__init__()

		self.zdot = 0.0
		self.z_d1 = P.z0
		self.err_d1 = 0.0
		self.integrator = 0.0

		self.a1 = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)
		self.a2 = 2.0 / (2*P.tau + P.Ts)


	def getForces(self, ref_input, states):
		z_r = ref_input
		z = states
		err = z_r - z

		# Differentiator
		self.zdot = self.a1*self.zdot + self.a2*(z - self.z_d1)
		#self.zdot = self.a1*self.zdot + self.a2*(err - self.err_d1)

		# Integrator
		self.integrator += (P.Ts/2.0)*(err + self.err_d1)

		self.err_d1 = err
		self.z_d1 = z

		F_unsat = P.kp*err - P.kd*self.zdot + P.ki*self.integrator
		F_sat = self.saturate(F_unsat, P.Fmax)

		# if P.ki != 0:
		# 	self.integrator += P.Ts/P.ki * (F_sat-F_unsat)

		self.forces = [F_sat]
		return F_sat



class MassSpringDamperControllerFullState(Controller):

	def __init__(self):

		super(MassSpringDamperControllerFullState, self).__init__()

		# Differentiator setup
		self.zdot = 0.0
		self.z_d1 = P.z0

		self.a1 = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)
		self.a2 = 2.0 / (2*P.tau + P.Ts)
		
		self.integrator = 0.0
		self.err_d1 = 0.0


	def getForces(self,ref_input, states):
		z_r = ref_input
		z = states

		# Differentiator
		self.zdot = self.a1*self.zdot + self.a2*(z - self.z_d1)
		self.z_d1 = z
		# Integrator
		err = z_r - z
		self.integrator += P.Ts/2.0 * (err + self.err_d1)
		self.err_d1 = err

		s = np.matrix([
			[z],
			[self.zdot]
		])
		# Without integrator
		#F = -P.K*s + P.kr*z_r
		# With integrator
		F = -P.K*s - P.ki*self.integrator + P.input_disturbance
		F = F.item(0)
		F_sat = self.saturate(F, P.Fmax)
		
		# anti windup
		if P.ki != 0:
			self.integrator += P.Ts/P.ki * (F_sat - F)

		self.forces = [F_sat]

		return self.forces[0]


class MassSpringDamperControllerObserver(Controller):

	def __init__(self):
		super(MassSpringDamperControllerObserver, self).__init__()
		self.xhat = np.matrix([
			[0.0],
			[0.0]
		])
		self.F_d1 = 0.0
		self.integrator = 0.0
		self.err_d1 = 0.0

	def getForces(self, ref_input, states):
		z_r = ref_input
		z = states

		N = 10
		for i in xrange(N):
			self.xhat += P.Ts/N*(P.A*self.xhat 
				         + P.B*self.F_d1 
				         + P.L*(z - P.C*self.xhat))

		err = z_r - self.xhat.item(0)
		self.integrator += P.Ts/2.0 * (err+self.err_d1)
		self.err_d1 = err

		F = -P.K*self.xhat - P.ki*self.integrator + P.input_disturbance
		F = F.item(0)
		F_sat = self.saturate(F, P.Fmax)
		self.F_d1 = F_sat

		if P.ki != 0:
			self.integrator += P.Ts/P.ki * (F_sat - F)

		self.forces = [F_sat]
		return F_sat

	def getObservedStates(self):
		return self.xhat.tolist()
