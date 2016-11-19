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
from ControllerPD import VTOLController

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['sim','plot'],
		help='Operating mode. sim: show animation. plot; show plot of response')
parser.add_argument('-i', nargs=2)
parser.add_argument('--save', action='store_true')
parser.add_argument('-t', type=float, help='Time duration of animation')
args = parser.parse_args()

hr = 0.0
zr = 0.0
if args.i:
	hr = float(args.i[0])
	zr = float(args.i[1])

controller = VTOLController()
animator = RotorVtolAnimation()
dynams = RotorVtolDynamics(controller)

zt = 0.8

def simulate():
	ts = 0.0
	te = 25.0
	if args.t: te = args.t
	t_Ts = P.Ts
	t_el = 0.1
	tp = 0.01

	t = ts
	count = 0
	n = 0

	animator.background()
	for _ in xrange(int(te/t_el)):
		plt.ion()
		plt.pause(0.001)
		tt = t + t_el
		for _ in xrange(int(t_el/t_Ts)):
			dynams.propogateDynamics([hr,zr])
			t += t_Ts
		plt.figure(animator.fig.number)
		output = dynams.Outputs()
		animator.draw(output, zt)
		if args.save:
			plt.savefig('figs/fig%d.jpg' % n)
			n += 1
	print controller.max_force


def plotResponse():
	te = 30.0
	if args.t: te = args.t
	F, tau = [], []
	h_resp = []
	z_resp = []
	t = np.linspace(0.0,te,te/P.Ts)
	for _ in t:
		dynams.propogateDynamics([hr,zr])
		forces = controller.state
		F.append(forces[0])
		tau.append(forces[1])
		output = dynams.Outputs()
		z_resp.append(output[0])
		h_resp.append(output[1])
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
	plt.plot(t, [hr for _ in t], 'b--')
	plt.plot(t, h_resp, 'r-')
	plt.title('Output h')
	plt.subplot(224)
	plt.grid()
	plt.plot(t, [zr for _ in t], 'b--')
	plt.plot(t, z_resp, 'g-')
	plt.title('Output z')
	plt.show(block=True)

if args.mode == 'sim':
	simulate()
elif args.mode == 'plot':
	plotResponse()

