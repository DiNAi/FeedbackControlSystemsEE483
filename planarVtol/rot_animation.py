import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import numpy as np
import params as P


class RotorVtolAnimation(object):

	def __init__(self):
		self.initflag = True
		self.fig, self.ax = plt.subplots()
		self.handles = [None] * 6
		plt.xlim(P.xlim)
		plt.ylim(P.ylim)
		self.ax.set_aspect('equal')


	def background(self):
		self.ground, = plt.plot(P.xlim, (0,0), 'g-')
		return self.ground

	
	def draw(self, u, zt):
		zv, h, theta = u
		self.set_rotation(theta)
		self.drawTarget(zt)
		self.drawPod(zv, h, theta)
		self.drawRotors(zv, h, theta)
		self.initflag = False
		return self.handles


	def set_rotation(self, theta):
		c = np.cos(theta)
		s = np.sin(theta)
		rot = np.array([[c, -s],[s,  c]])
		self.rotation = rot


	def drawTarget(self, zt):
		xs = [zt-P.tw/2.0, zt+P.tw/2.0] 
		xs += xs[-1::-1]
		ys = [0,0] + [P.th] * 2
		xy = zip(xs,ys)

		if self.initflag:
			target_patch = mpatch.Polygon(xy, color='purple')
			self.handles[0] = target_patch
			self.ax.add_patch(target_patch)
		else:
			self.handles[0].set_xy(xy)


	def drawPod(self, zv, h, theta):
		t = np.array([zv, h])
		xs = [-P.rw/2.0, P.rw/2.0]
		xs += xs[-1::-1]
		ys = [-P.rh/2.0]*2 + [P.rh/2.0] * 2
		xy = zip(xs,ys)
		xy = [self.rotation.dot(p)+t for p in xy]

		if self.initflag:
			pod_patch = mpatch.Polygon(xy, color='navy')
			self.handles[1] = pod_patch
			self.ax.add_patch(pod_patch)
		else:
			self.handles[1].set_xy(xy)


	def drawRotors(self, zv, h, theta):
		t = np.array([zv, h])
		xs = [-P.d, P.d]
		ys = [0,0]
		xy = zip(xs,ys)
		xy = np.array([self.rotation.dot(p)+t for p in xy])

		if self.initflag:
			line, = self.ax.plot(xy[:,0], xy[:,1], color='navy')
			lrot_patch = mpatch.Circle(xy[0], P.rr, color='navy')
			rrot_patch = mpatch.Circle(xy[1], P.rr, color='navy')
			self.handles[2:5] = line,lrot_patch,rrot_patch
			self.ax.add_patch(lrot_patch)
			self.ax.add_patch(rrot_patch)
		else:
			self.handles[2].set_xdata(xy[:,0])
			self.handles[2].set_ydata(xy[:,1])
			self.handles[3].center = xy[0]
			self.handles[4].center = xy[1]
			


if __name__ == '__main__':
	animator = RotorVtolAnimation()
	animator.background()
	animator.draw([1, 1.4, np.pi/6.0])
	plt.show()
