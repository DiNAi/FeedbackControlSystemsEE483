#!/usr/bin/python
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import matplotlib.animation as ani
import params as P
import argparse
from rot_animation import RotorVtolAnimation
from rot_dynamics import RotorVtolDynamics
import vtol_controller as ctrl
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import signalGenerator as sigGen

control_types = ['pd', 'pid', 'full-state', 'observer']

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['sim','plot'],
		help='Operating mode. sim: show animation. plot; show plot of response')
parser.add_argument('-i', nargs=2)
parser.add_argument('--save', action='store_true')
parser.add_argument('-t', type=float, help='Time duration of animation')
parser.add_argument('-c', type=str, choices=control_types,
		help='Controller type: %s' % control_types)

args = parser.parse_args()

hr = 0.05
zr = 0.0
if args.i:
	hr = float(args.i[0])
	zr = float(args.i[1])

controller = None
if args.c == 'pd':
	controller = ctrl.VTOLControllerPD()
elif args.c == 'pid':
	controller = ctrl.VTOLControllerPID()
elif args.c == 'full-state':
	controller = ctrl.VTOLControllerFullState()
elif args.c == 'observer':
	controller = ctrl.VTOLControllerObserver()
else:
	controller = ctrl.VTOLControllerPID()

dynams = RotorVtolDynamics(controller)
animator = RotorVtolAnimation()

zt = 0.8

def simulate():
	t_el = 0.1
	te = 25.0
	if args.t: te = args.t

	n = 0
	h_hz = 0.1
	z_hz = 0.05
	hr = sigGen.squareWave(1, 2, h_hz, P.Ts)
	zr = sigGen.squareWave(-1.5, 1.5, z_hz, P.Ts)

	animator.background()
	for _ in xrange(int(te/t_el)):
		plt.pause(0.001)
		for _ in xrange(int(t_el/P.Ts)):
			dynams.propogateDynamics([next(hr),next(zr)])
		plt.figure(animator.fig.number)
		output = dynams.Outputs()
		animator.draw(output, zt)
		if args.save:
			plt.savefig('figs/fig%d.jpg' % n)
			n += 1


def plotResponse():
	te = 30.0
	if args.t: te = args.t
	h_hz = 0.1
	z_hz = 0.05
	hr = sigGen.squareWave(1, 2, h_hz, P.Ts)
	zr = sigGen.squareWave(-1.5, 1.5, z_hz, P.Ts)
	F, tau = [], []
	h_ref, h_resp = [], []
	z_ref, z_resp = [], []
	z_obs, h_obs = [], []
	t = np.linspace(0.0,te,te/P.Ts)
	for _ in t:
		h_ref.append(next(hr))
		z_ref.append(next(zr))
		dynams.propogateDynamics([h_ref[-1],z_ref[-1] + P.wind])
		forces = controller.forces
		F.append(forces[0])
		tau.append(forces[1])
		output = dynams.Outputs()
		z_resp.append(output[0])
		h_resp.append(output[1])
		if args.c == 'observer':
			z_obs.append(controller.xhat_lat.item(0))
			h_obs.append(controller.xhat_lon.item(0))
	plt.figure(1)
	plt.subplot(221)
	plt.grid()
	plt.plot(t, F, 'r-')
	plt.title('Input F')
	plt.subplot(223)
	plt.grid()
	plt.plot(t, tau, 'g-')
	plt.title('Input tau')
	plt.subplot(222)
	plt.grid()
	plt.plot(t, h_ref, 'b--')
	plt.plot(t, h_resp, 'r-')
	plt.title('Output h')
	plt.subplot(224)
	plt.grid()
	plt.plot(t, z_ref, 'b--')
	plt.plot(t, z_resp, 'g-')
	plt.title('Output z')

	if args.c == 'observer':
		plt.figure(2)
		plt.subplot(221)
		plt.grid()
		plt.plot(t, h_resp, 'r-')
		plt.plot(t, h_obs, 'b--')
		plt.title('Actual vs observed h')
		plt.subplot(222)
		plt.grid()
		plt.plot(t, z_resp, 'r-')
		plt.plot(t, z_obs, 'b--')
		plt.title('Actual vs observed z')
		plt.subplot(223)
		plt.grid()
		plt.plot(t, np.array(h_resp)-np.array(h_obs), 'r-')
		plt.title('Error (h_actual - h_observed)')
		plt.subplot(224)
		plt.grid()
		plt.plot(t, np.array(z_resp)-np.array(z_obs), 'r-')
		plt.title('Error (z_actual - z_observed)')

	plt.show(block=True)


if args.mode == 'sim':
	simulate()
elif args.mode == 'plot':
	plotResponse()

