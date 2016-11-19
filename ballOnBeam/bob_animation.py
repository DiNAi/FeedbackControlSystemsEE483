import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import matplotlib.transforms as mtrans
import numpy as np
import params as P


class BallOnBeamAnimation(object):

	def __init__(self):
		self.initflag = True
		self.fig, self.ax = plt.subplots()
		self.ax.set_aspect('equal')
		self.handles = [None] * 2
		plt.xlim(P.xlim)
		plt.ylim(P.ylim)


	def background(self):
		self.level, = plt.plot(P.xlim, (0,0), 'b:')
		self.pivot, = plt.plot((0,0), (0,P.ylim[0]), 'r-', lw=2)
		return (self.level, self.pivot)


	def draw(self, u):
		z = u[0]
		theta = u[1]
		self.drawBeam(theta)
		self.drawBall(z, theta)
		self.initflag = False
		return self.handles


	def drawBeam(self, theta):
		xs = [0, P.l * np.cos(theta)]
		ys = [0, P.l * np.sin(theta)]
		if self.initflag:
			beam_patch, = plt.plot(xs,ys,lw=3,color='black')
			self.handles[0] = beam_patch
		else:
			self.handles[0].set_xdata(xs)
			self.handles[0].set_ydata(ys)


	def drawBall(self, z, theta):
		x = z * np.cos(theta)
		y = z * np.sin(theta) + P.r + 0.010
		if self.initflag:
			ball_patch = mpatch.Circle((x,y), P.r, color='green')
			self.handles[1] = ball_patch
			self.ax.add_patch(ball_patch)
		else:
			self.handles[1].center = (x,y)



if __name__ == '__main__':
	animator = BallOnBeamAnimation()
	z = 0.5
	theta = np.pi/4.0
	animator.background()
	animator.draw((z,theta))
	plt.show()

