import numpy as np
from controlTrace import ControlTrace

class Controller(object):

	def __init__(self):
		self.forces = [0.0]


	def saturate(self, F, limit):
		if abs(F) > limit:
			F = limit*np.sign(F)
		return F


	def getForces(self, ref_input, state):
		pass



class ControllerPD(Controller):

	def __init__(self, kp, kd, Feq=0.0):
		super(BallOnBeamControllerPD, self).__init__()
		
		self.kp = float(kp)
		self.kd = float(kd)
		self.Feq = float(Feq)


	def loopOutput(self, ref_input, state, state_derivative, kp, kd):
		return (kp * (ref_input - state)) - (kd * state_derivative)


class ControllerObserver(Controller):

	def __init__(self, num_states, A, B, C, L, K, ki, Fmax):
		super(ControllerObserver, self).__init__()
		
		self.xhat = np.matrix([[0.0] for _ in xrange(num_states)])
		self.A = A
		self.B = B
		self.C = C
		self.L = L
		self.K = K
		self.ki = ki

		self.F_d1 = 0.0
		self.integrator = 0.0
		self.err_d1 = 0.0

		self.Fmax = Fmax


	def iterate_xhat(self, state):
		N = 10
		for i in xrange(N):
			self.xhat += (self.Ts/N*(self.A*self.xhat 
				         + self.B*self.F_d1 
				         + self.L*(state - self.C*self.xhat)))


	def update_integrator(self, ref_input, obs_state):
		err = ref_input - obs_state
		self.integrator += self.Ts/2.0 * (err + self.err_d1)
		self.err_d1 = err


	def computeForce(self, input_disturbance):
		F = -self.K*self.xhat - self.ki*self.integrator + input_disturbance
		F = F.item(0)
		F_sat = self.saturate(F, self.Fmax)
		self.F_d1 = F_sat
		return (F, F_sat)


	def anti_windup(self, F, F_sat):
		if self.ki != 0:
			self.integrator += self.Ts/self.ki * (F_sat - F)


	def getForces(self, ref_input, ref_output):
		z_r = ref_input
		z = states

		self.iterate_xhat(ref_output)
		self.update_integrator(ref_input, self.xhat.item(0))
		F, F_sat = computeForce(0.25) 
		self.anti_windup(F, F_sat)

		self.forces = [F_sat]
		return F_sat

	def getObservedStates(self):
		return self.xhat.tolist()