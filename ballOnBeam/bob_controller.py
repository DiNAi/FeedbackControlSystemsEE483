import params as P
import numpy as np
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from controller import Controller


class BallOnBeamControllerPD(Controller):

	def __init__(self):
		super(BallOnBeamControllerPD, self).__init__()

		self.M1 = P.m1*P.g/P.l
		self.M2 = P.m2*P.g/(2.0)

		self.max_force = 0


	def getForces(self, ref_input, states):
		z_r = ref_input[0]

		z = states.item(0)
		theta = states.item(1)
		zdot = states.item(2)
		thetadot = states.item(3)
		
		Feq = self.M1*z + self.M2
		th_r = P.kp_z*(z_r - z) - P.kd_z*zdot
		Ftilde = P.kp_th*(th_r - theta) - P.kd_th*thetadot
		F = Ftilde + Feq

		F = self.saturate(F, self.Fmax)
		self.forces = [F]
		return self.forces



class BallOnBeamControllerPID(BallOnBeamControllerPD):

	def __init__(self):
		super(BallOnBeamControllerPID, self).__init__()

		self.diff_theta = 0.0
		self.int_theta = 0.0
		self.theta_d1 = P.theta0
		self.err_theta_d1 = 0.0

		self.diff_z = 0.0
		self.int_z = 0.0
		self.z_d1 = P.z0
		self.err_z_d1 = 0.0

		self.a1 = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)
		self.a2 = 2.0 / (2*P.tau + P.Ts)


	def getForces(self, ref_input, states):
		z_r = ref_input[0]
		z = states.item(0)
		theta = states.item(1)

		# OUTER LOOP CONTROL
		err = z_r - z
		# z differentiator
		self.diff_z = self.a1*self.diff_z + self.a2*(z - self.z_d1)
		# z integrator
		self.int_z += (P.Ts/2.0)*(err+self.err_z_d1)
		# update z params
		self.err_z_d1 = err
		self.z_d1 = z
		# calculate theta_r using PID
		theta_r_unsat = P.kp_z*err - P.kd_z*self.diff_z + P.ki_z*self.int_z
		#theta_r_sat = self.saturate(theta_r_unsat, self.theta_max)
		# Anti wind-up
		# if P.ki_z != 0:
		# 	self.int_z += P.Ts/P.ki_z*(theta_r_sat-theta_r_unsat)

		# INNER LOOP CONTROL
		err = theta_r_unsat - theta
		# theta differentiator
		self.diff_theta = self.a1*self.diff_theta + self.a2*(theta - self.theta_d1)
		# theta integrator
		self.int_theta += (P.Ts/2.0)*(err+self.err_theta_d1)
		# update theta params
		self.err_theta_d1 = err
		self.theta_d1 = theta
		# calculate F using PID
		Feq = self.M1*z + self.M2
		F_unsat = P.kp_th*err - P.kd_th*self.diff_theta + P.ki_th*self.int_theta + Feq
		F_sat = self.saturate(F_unsat, P.Fmax)
		# Anti wind-up
		# if P.ki_th != 0:
		# 	self.int_theta += P.Ts/P.ki_th*(F_sat - F_unsat)

		self.forces = [F_sat]
		return [F_sat]


class BallOnBeamControllerFullState(Controller):

	def __init__(self):
		super(BallOnBeamControllerFullState,self).__init__()

		self.zdot = 0.0
		self.z_d1 = P.z0

		self.thetadot = 0.0
		self.theta_d1 = 0.0

		self.integrator = 0.0
		self.err_d1 = 0.0

		self.a1 = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)
		self.a2 = 2.0 / (2*P.tau + P.Ts)
		self.Ts_half = P.Ts/2.0


	def getForces(self, ref_input, states):
		z_r = ref_input[0]
		z = states.item(0)
		theta = states.item(1)

		self.zdot = self.a1*self.zdot + self.a2*(z - self.z_d1)
		self.thetadot = self.a1*self.thetadot + self.a2*(theta - self.theta_d1)
		self.z_d1 = z
		self.theta_d1 = theta

		err = z_r - z
		self.integrator += self.Ts_half*(err + self.err_d1)
		self.err_d1 = err

		s = np.matrix([
			[z - P.z0],
			[theta],
			[self.zdot],
			[self.thetadot]
		])
		# Without integrator
		#F = P.Feq - P.K*s + P.kr*(z_r-P.z0)
		# With integrator
		F = P.Feq - P.K1*s - P.ki*self.integrator + P.input_disturbance
		F = F.item(0)
		F_sat = self.saturate(F, P.Fmax)

		# anti windup
		if P.ki != 0:
			self.integrator += P.Ts/P.ki * (F_sat - F)
		
		self.forces = [F_sat]

		return self.forces


class BallOnBeamControllerObserver(Controller):

	def __init__(self):
		super(BallOnBeamControllerObserver, self).__init__()
		self.xhat = np.matrix([
			[P.z0],
			[0.0],
			[0.0],
			[0.0]
		])
		self.xeq = np.matrix([
			[P.z0],
			[P.theta0],
			[P.zdot0],
			[P.thetadot0]
		])
		self.F_d1 = P.Feq
		self.integrator = 0.0
		self.err_d1 = 0.0


	def getForces(self, ref_input, states):

		N = 10
		for i in xrange(N):
			self.xhat += P.Ts/N*(P.A*(self.xhat -self.xeq)
				         + P.B*(self.F_d1 - P.Feq)
				         + P.L*(states[:2] - P.C*self.xhat))

		err = ref_input[0] - self.xhat.item(0)
		self.integrator += P.Ts/2.0 * (err+self.err_d1)
		self.err_d1 = err

		F = P.Feq - P.K1*(self.xhat-self.xeq) - P.ki*self.integrator + P.input_disturbance
		F = F.item(0)
		F_sat = self.saturate(F, P.Fmax)
		self.F_d1 = F_sat

		if P.ki != 0:
			self.integrator += P.Ts/P.ki * (F_sat - F)

		self.forces = [F_sat]
		return [F_sat]

	def getObservedStates(self):
		return self.xhat.tolist()