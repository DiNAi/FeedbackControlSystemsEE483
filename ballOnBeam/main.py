#!/usr/bin/python
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import matplotlib.animation as ani
import params as P
import argparse
from bob_animation import BallOnBeamAnimation
from bob_dynamics import BallOnBeamDynamics
import bob_controller as ctrl
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import signalGenerator as sigGen

control_types = ['pd', 'pid', 'full-state', 'observer']

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

zr = 0.0
if args.i:
	zr = args.i
else:
	zr = 0.0

controller = None
if args.c == 'pd':
	controller = ctrl.BallOnBeamControllerPD()
elif args.c == 'pid':
	controller = ctrl.BallOnBeamControllerPID()
elif args.c == 'full-state':
	controller = ctrl.BallOnBeamControllerFullState()
elif args.c == 'observer':
	controller = ctrl.BallOnBeamControllerObserver()
else:
	controller = ctrl.BallOnBeamControllerPID()
	
dynams = BallOnBeamDynamics(controller)
animator = BallOnBeamAnimation()

def simulate():
	global zr
	te = 6.0  # Ending time in seconds
	t_el = 0.1  # Time between animation updates
	if args.t: te = args.t # Optional end time specification

	n = 0
	hz = 0.1
	z_ref = sigGen.squareWave(0.1, 0.4, hz, P.Ts)
	
	plt.figure(animator.fig.number)
	animator.background()
	for _ in xrange(int(te / t_el)):
		plt.pause(0.001)
		for _ in xrange(int(t_el / P.Ts)):
			dynams.propogateDynamics([next(z_ref)])
		output = dynams.Outputs()
		animator.draw(output)
		if args.save:
			plt.savefig('figs/fig%d.jpg' % n)
			n += 1


def plotResponse():
	force = []
	reference = []
	z_resp = []
	z_obs = []
	theta_resp = []
	theta_obs = []
	te = 20.0
	if args.t: te = args.t
	hz = 0.1
	z_ref = sigGen.squareWave(0.1, 0.4, hz, P.Ts)
	t = np.linspace(0.0,te,1/P.Ts*te)
	for _ in t:
		zr = next(z_ref)
		reference.append(zr)
		dynams.propogateDynamics([zr])
		output = dynams.Outputs()
		z_resp.append(output[0])
		theta_resp.append(output[1])
		F = controller.forces[0]
		force.append(F)
		if args.c == 'observer':
			z_obs.append(controller.xhat.item(0))
			theta_obs.append(controller.xhat.item(1))

	plt.figure(1)
	plt.subplot(311)
	plt.plot(t, force, 'b-')
	plt.title('System input F')
	plt.grid()

	plt.subplot(312)
	plt.plot(t, theta_resp, 'r-')
	plt.title('System response Theta')
	plt.grid()

	plt.subplot(313)
	plt.plot(t, reference, 'b--')
	plt.plot(t, z_resp, 'r-')
	plt.title('System response Z')
	plt.grid()

	if args.c == 'observer':
		plt.figure(2)
		plt.subplot(221)
		plt.plot(t, z_resp, 'r-')
		plt.plot(t, z_obs, 'b--')
		plt.title('Actual vs Observed z')
		plt.grid()

		plt.subplot(222)
		plt.plot(t, theta_resp, 'r-')
		plt.plot(t, theta_obs, 'b--')
		plt.title('Actual vs Observed theta')
		plt.grid()

		plt.subplot(223)
		plt.plot(t, np.array(z_resp)-np.array(z_obs), 'r-')
		plt.title('Error (actual - observed)')
		plt.grid()

		plt.subplot(224)
		plt.plot(t, np.array(theta_resp)-np.array(theta_obs), 'r-')
		plt.title('Error (actual - observed)')
		plt.grid()

	plt.show()

if args.mode == 'sim':
	simulate()
elif args.mode == 'plot':
	plotResponse()

