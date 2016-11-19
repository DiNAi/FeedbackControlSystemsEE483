#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import params as P
import argparse
from msd_animation import MassSpringDamperAnimation
from msd_dynamics import MassSpringDamperDynamics
import msd_controller as ctrl
import sys,os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import signalGenerator as sigGen

control_types = ['pid', 'full-state', 'observer']

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

def simulate():
	te = 12.0
	t_el = 0.1
	if args.t: te = args.t

	hz = 0.05
	zr = sigGen.squareWave(-0.5, 1.5, hz, P.Ts)
	n = 0

	plt.figure(animator.fig.number)
	animator.background()
	for _ in xrange(int(te / t_el)):
		plt.pause(0.001)
		for _ in xrange(int(t_el / P.Ts)):
			s = dynams.States()
			dynams.propogateDynamics([next(zr)])
		output = dynams.Outputs()
		animator.draw(output)
		if args.save:
			plt.savefig('figs/fig%d.jpg' % n)
			n += 1


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
