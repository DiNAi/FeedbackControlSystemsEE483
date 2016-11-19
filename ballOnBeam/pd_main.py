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
from ControllerPD import BallOnBeamController

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['sim','plot'],
		help='Operating mode. sim: show animation. plot; show plot of response')
parser.add_argument('-i', type=float, 
		help='Input to system: the desired position of the mass')
parser.add_argument('--save', action='store_true', default='False',
		help='Save the animation as a series of frames')
parser.add_argument('-t', type=float,
		help='Time duration of animation')
args = parser.parse_args()

zr = 0.0
if args.i:
	zr = args.i
else:
	zr = 0.0

animator = BallOnBeamAnimation()
controller = BallOnBeamController()
dynams = BallOnBeamDynamics(controller)

def simulate():
	global zr
	ts = 0.0  # Starting time in seconds
	te = 6.0  # Ending time in seconds
	if args.t: te = args.t # Optional end time specification
	t_Ts = P.Ts # Time step in seconds
	t_el = 0.1  # Time between animation updates

	t = ts
	n = 0
	u = [zr]
	
	animator.background()
	for _ in xrange(int(te / t_el)):
		if int(t) % 100 < 50: zr = 0.25 + 0.15
		else:                 zr = 0.25 - 0.15
		plt.pause(0.001)
		tt = t + t_el
		for _ in xrange(int(t_el / t_Ts)):
			s = dynams.States()
			dynams.propogateDynamics(u)
			t += t_Ts
		plt.figure(animator.fig.number)
		output = dynams.Outputs()
		animator.draw(output)
		if args.save:
			plt.savefig('figs/fig%d.jpg' % n)
			n += 1


def plotResponse():
	force = []
	z_resp = []
	theta_resp = []
	te = 20.0
	if args.t: te = args.t
	t = np.linspace(0.0,te,1/P.Ts*te)
	u = [zr]
	for _ in t:
		s = dynams.States()
		F = controller.forces[0]
		force.append(F)
		dynams.propogateDynamics(u)
		output = dynams.Outputs()
		z_resp.append(output[0])
		theta_resp.append(output[1])

	plt.subplot(131)
	plt.plot(t, force, 'b-')
	plt.title('System input F')
	plt.grid()

	plt.subplot(132)
	plt.plot(t, [zr for x in t], 'b--')
	plt.plot(t, z_resp, 'r-')
	plt.title('System response Z')
	plt.grid()

	plt.subplot(133)
	plt.plot(t, theta_resp, 'r-')
	plt.title('System response Theta')
	plt.grid()

	plt.show()

if args.mode == 'sim':
	simulate()
elif args.mode == 'plot':
	plotResponse()

