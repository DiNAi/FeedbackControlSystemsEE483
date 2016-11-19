#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import argparse
from Controller import Controller
import signalGenerator as sigGen
# NEED GENERIC DYNAMICS AND ANIMATION CLASSES
# from msd_animation import MassSpringDamperAnimation
# from msd_dynamics import MassSpringDamperDynamics
# import sys,os
# sys.path.insert(1, os.path.join(sys.path[0], '..'))


class ControlSimulator(object):

	control_types = ['pd', 'pid', 'full-state', 'observer']

	def __init__(self, controller, dynamics, animator, **kwargs):
		"""Valid kwargs: t_end, t_elapse, Ts, signals,
		"""

		self.controller = controller
		self.dynamics = dynamics
		self.animator = animator

		self.t_end = 15.0
		self.t_elapse = 0.1
		self.Ts = 0.01

		if 't_end' in kwargs:
			self.t_end = kwargs['t_end']
		if 't_elapse' in kwargs:
			self.t_elapse = kwargs['t_elapse']
		if 'Ts' in kwargs:
			self.Ts = kwargs['Ts']
		if 'signal' in kwargs:
			self.signals = kwargs['signals']
		else:
			hz = 0.01
			self.signals = [sigGen.squareWave(-0.5, 0.5, hz, self.Ts)]


	def animate(self):
		te = self.t_end
		t_el = self.t_elapse

		plt.figure(self.animator.fig.number)
		self.animator.background()
		for _ in xrange(int(te / t_el)):
			plt.pause(0.001)
			for _ in xrange(int(t_el / P.Ts)):
				ref_input = [next(x) for x in self.signals]
				self.dynamics.propogateDynamics(ref_input)
			output = self.dynamics.Outputs()
			self.animator.draw(output)
			

	def plot(self):
		te = self.t_end
		t_el = self.t_elapse
		Ts = self.Ts

		reference = []
		forces = []
		outputs = []
		observed = []

		is_observed = False
		try:
			xhat = self.controller.xhat
			is_observed = True
		except AttributeError:
			is_observed = False

		for _ in t:
			ref_input = [next(x) for x in self.signals]
			self.dynamics.propogateDynamics(ref_input)
			output = self.dynamics.Outputs()
			reference.append(ref_input)
			forces.append(controller.forces)
			outputs.append(output)
			if is_observed:
				observed.append(controller.xhat.tolist())

		t = np.linspace(0.0,te,te / P.Ts)
		plt.figure(1)
		plt.subplot(211)
		plt.plot(t, force, 'r-')
		plt.grid()
		plt.title('Force')

		plt.subplot(212)
		plt.plot(t, ref, 'b--')
		plt.plot(t, pos, 'r-')
		plt.grid()
		plt.title('Position of mass')

		if args.c == 'observer':
			plt.figure(2)
			plt.subplot(211)
			plt.plot(t, pos, 'r-')
			plt.plot(t, obs, 'b--')
			plt.grid()
			plt.title('Observer vs. Actual')

			plt.subplot(212)
			plt.plot(t, np.array(pos) - np.array(obs), 'r-')
			plt.grid()
			plt.title('Error (actual - observed)')

		plt.show(block=True)	



parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['sim','plot'],
		help='Operating mode. sim: show animation. plot; show plot of response')
parser.add_argument('-i', type=float, 
		help='Input to system: the desired position of the mass')
parser.add_argument('--save', action='store_true', default='False',
		help='Save the animation as a series of frames')
parser.add_argument('-t', type=float,
		help='Time duration of animation')
parser.add_argument('-c', type=str, choices=control_types,
		help='Controller type: %s' % control_types)
args = parser.parse_args()

if args.i:
	zr = args.i
else:
	zr = 0.0

controller = None
if args.c == 'pid':
	controller = ctrl.MassSpringDamperControllerPID()
elif args.c == 'full-state':
	controller = ctrl.MassSpringDamperControllerFullState()
elif args.c == 'observer':
	controller = ctrl.MassSpringDamperControllerObserver()
else:
	controller = ctrl.MassSpringDamperControllerPID()

dynams = MassSpringDamperDynamics(controller)
animator = MassSpringDamperAnimation()


def plotResponse():
	te = 12.0
	if args.t: te = args.t
	force = []
	pos = []
	ref = []
	obs = []
	hz = 0.05
	z_ref = sigGen.squareWave(-0.5, 1.5, hz, P.Ts)
	t = np.linspace(0.0,te,te / P.Ts)
	for _ in t:
		s = dynams.States()
		zr = next(z_ref)
		dynams.propogateDynamics([zr])
		output = dynams.Outputs()
		ref.append(zr)
		force.append(controller.forces[0])
		pos.append(output)
		if args.c == 'observer':
			obs.append(controller.xhat.item(0))

	plt.figure(1)
	plt.subplot(211)
	plt.plot(t, force, 'r-')
	plt.grid()
	plt.title('Force')

	plt.subplot(212)
	plt.plot(t, ref, 'b--')
	plt.plot(t, pos, 'r-')
	plt.grid()
	plt.title('Position of mass')

	if args.c == 'observer':
		plt.figure(2)
		plt.subplot(211)
		plt.plot(t, pos, 'r-')
		plt.plot(t, obs, 'b--')
		plt.grid()
		plt.title('Observer vs. Actual')

		plt.subplot(212)
		plt.plot(t, np.array(pos) - np.array(obs), 'r-')
		plt.grid()
		plt.title('Error (actual - observed)')

	plt.show(block=True)

if args.mode == 'sim':
	simulate()
elif args.mode == 'plot':
	plotResponse()
