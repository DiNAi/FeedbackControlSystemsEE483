import params as P
import numpy as np
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from controller import Controller


class VTOLControllerPD(Controller):

	def __init__(self):
		super(VTOLControllerPD, self).__init__()

		self.max_force = 0
		self.state = [0, 0]


	def getForces(self, ref_input, states):
		h_r = ref_input[0]
		z_r = ref_input[1]
		z = states.item(0)
		h = states.item(1)
		theta = states.item(2)
		zdot = states.item(3)
		hdot = states.item(4)
		thetadot = states.item(5)
		
		Ftilde = P.kp_h*(h_r - h) - P.kd_h*hdot
		F = self.saturate(Ftilde + P.Feq, P.Fmax)
		if F > self.max_force:
			self.max_force = F

		th_r = P.kp_z*(z_r - z) - P.kd_z*zdot
		tau = P.kp_th*(th_r - theta) - P.kd_th*thetadot
		self.state = [F, tau]
		return self.state


class VTOLControllerPID(VTOLControllerPD):

	def __init__(self):
		super(VTOLControllerPID,self).__init__()

		# theta
		self.diff_theta = 0.0
		self.int_theta = 0.0
		self.theta_d1 = P.theta0
		self.err_theta_d1 = 0.0
		self.theta_max = np.pi
		# z
		self.diff_z = 0.0
		self.int_z = 0.0
		self.z_d1 = P.z0
		self.err_z_d1 = 0.0
		# h
		self.diff_h = 0.0
		self.int_h = 0.0
		self.h_d1 = P.h0
		self.err_h_d1 = 0.0

		self.a1 = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)
		self.a2 = 2.0 / (2*P.tau + P.Ts)


	def getForces(self, ref_input, states):
		h_r, z_r = ref_input
		z = states.item(0)
		h = states.item(1)
		theta = states.item(2)

		# LONGITUDINAL CONTROL
		err = h_r - h
		# h differentiator
		self.diff_h = self.a1*self.diff_h + self.a2*(h - self.h_d1)
		# h integrator
		self.int_h += (P.Ts/2.0)*(err+self.err_h_d1)
		# update h params
		self.err_h_d1 = err
		self.h_d1 = h
		# Calculate the force
		F_unsat = P.kp_h*err - P.kd_h*self.diff_h + P.ki_h*self.int_h + P.Feq
		F_sat = self.saturate(F_unsat, P.Fmax)
		# anti wind-up
		# if P.ki_h != 0:
		# 	self.int_z += P.Ts/P.ki_h*(F_sat-F_unsat)

		# LATERAL CONTROL
		# OUTER LOOP
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
		theta_r_sat = self.saturate(theta_r_unsat, self.theta_max)

		# INNER LOOP
		err = theta_r_sat - theta
		# theta differentiator
		self.diff_theta = self.a1*self.diff_theta + self.a2*(theta - self.theta_d1)
		# theta integrator
		self.int_theta += (P.Ts/2.0)*(err+self.err_theta_d1)
		# update theta params
		self.err_theta_d1 = err
		self.theta_d1 = theta
		# calculate F using PID
		tau_unsat = P.kp_th*err - P.kd_th*self.diff_theta + P.ki_th*self.int_theta
		tau_sat = self.saturate(tau_unsat, P.tau_max)

		self.max_force = max(self.max_force, F_sat)
		self.state = [F_sat, tau_sat]
		return self.state


class VTOLControllerFullState(Controller):

	def __init__(self):
		super(VTOLControllerFullState, self).__init__()

		self.state = [0, 0]

		self.zdot = 0.0
		self.hdot = 0.0
		self.thetadot = 0.0
		self.z_d1 = P.z0
		self.h_d1 = P.h0
		self.theta_d1 = P.theta0

		self.integrator_lon = 0.0
		self.err_lon_d1 = 0.0
		self.integrator_lat = 0.0
		self.err_lat_d1 = 0.0

 		self.a1 = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)
		self.a2 = 2.0 / (2*P.tau + P.Ts)
		self.Ts_half = P.Ts/2.0


	def getForces(self, ref_input, states):
		h_r, z_r = ref_input
		z = states.item(0)
		h = states.item(1)
		theta = states.item(2)		

		self.hdot = self.a1*self.hdot + self.a2*(h - self.h_d1)
		self.h_d1 = h

		self.zdot = self.a1*self.zdot + self.a2*(z - self.z_d1)
		self.z_d1 = z

		self.thetadot = self.a1*self.thetadot + self.a2*(theta - self.theta_d1)
		self.theta_d1 = theta

		err = h_r - h
		self.integrator_lon += self.Ts_half*(err + self.err_lon_d1)
		self.err_lon_d1 = err

		err = z_r - z
		self.integrator_lat += self.Ts_half*(err + self.err_lat_d1)
		self.err_lat_d1 = err

		s_lon = np.matrix([
			[h-P.h0],
			[self.hdot]
		])

		s_lat = np.matrix([
			[z - P.z0],
			[theta],
			[self.zdot],
			[self.thetadot]
		])

		# without integrator
		#F = P.Feq - P.K_lon*s_lon + P.kr_lon*(h_r-P.h0)
		# with integrator
		F = P.Feq - P.K1_lon*s_lon - P.ki_lon*self.integrator_lon
		F = F.item(0)
		F_sat = self.saturate(F, P.Fmax)
		# anti windup
		if P.ki_lon != 0:
			self.integrator_lon += P.Ts/P.ki_lon * (F_sat - F)

		#tau = -P.K_lat*s_lat + P.kr_lat*(z_r-P.z0)
		tau = -P.K1_lat*s_lat - P.ki_lat*self.integrator_lat
		tau = tau.item(0)

		self.forces = [F_sat, tau]

		return self.forces


class VTOLControllerObserver(Controller):

	def __init__(self):
		super(VTOLControllerObserver, self).__init__()
		self.xeq_lon = np.matrix([
			[P.h0],
			[P.hdot0]
		])
		self.xhat_lon = self.xeq_lon.copy()
		self.xeq_lat = np.matrix([
			[P.z0],
			[P.theta0],
			[P.zdot0],
			[P.thetadot0]
		])
		self.xhat_lat = self.xeq_lat.copy()

		self.F_d1 = P.Feq
		self.tau_d1 = 0.0
		self.integrator_lon = 0.0
		self.integrator_lat = 0.0
		self.err_d1_lon = 0.0
		self.err_d1_lat = 0.0


	def getForces(self, ref_input, states):
		h_r, z_r = ref_input
		z = states.item(0)
		h = states.item(1)
		theta = states.item(2)

		N = 10
		for i in xrange(N):
			Ts = P.Ts/N
			self.xhat_lon += Ts*(P.A_lon*(self.xhat_lon - self.xeq_lon)
				         + P.B_lon*(self.F_d1 - P.Feq)
				         + P.L_lon*([h] - P.C_lon*self.xhat_lon))

			self.xhat_lat += Ts*(P.A_lat*(self.xhat_lat - self.xeq_lat)
						 + P.B_lat*self.tau_d1
						 + P.L_lat*([[z],[theta]] - P.C_lat*self.xhat_lat))


		err = h_r - self.xhat_lon.item(0)
		self.integrator_lon += P.Ts/2.0 * (err+self.err_d1_lon)
		self.err_d1_lon = err

		err = z_r - self.xhat_lat.item(0)
		self.integrator_lat += P.Ts/2.0 * (err+self.err_d1_lat)
		self.err_d1_lat = err

		F = P.Feq - P.K1_lon*(self.xhat_lon-self.xeq_lon) - P.ki_lon*self.integrator_lon + P.wind
		F = F.item(0)
		F_sat = self.saturate(F, P.Fmax)
		self.F_d1 = F_sat

		tau = -P.K1_lat*(self.xhat_lat-self.xeq_lat) - P.ki_lat*self.integrator_lat
		tau = tau.item(0)
		self.tau_d1 = tau

		if P.ki_lon != 0:
			self.integrator_lon += P.Ts/P.ki_lon * (F_sat - F)

		self.forces = [F_sat, tau]
		return [F_sat, tau]

	def getObservedStates(self):
		return self.xhat.tolist()