import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import numpy as np
import params as P


class MassSpringDamperAnimation(object):

	def __init__(self):
		self.initflag = True				# Indicates initialization stage
		self.fig, self.ax = plt.subplots()  # Init figure and axis
		self.ax.set_aspect('equal')
		self.handles = [None] * 3		   # List of handles to drawing objects
		plt.xlim(P.xlim)					# Set the x axis
		plt.ylim(P.ylim)					# Set the y axis


	def background(self):
		self.base, = plt.plot(P.xlim, (0,0), 'b-')		# Plot the baseline
		# Plot the wall
		self.wall, = plt.plot((P.wall_pos,P.wall_pos), (0,P.ylim[1]), 'b-', lw=2) 
		return (self.base, self.wall)


	def draw(self, z):
		self.drawMass(z)
		self.drawSpring(z)
#		self.drawDamper(z)
		self.initflag = False
		return self.handles


	def drawMass(self, z):
		xy = (z - P.w/2.0, 0)

		if self.initflag:
			mass_patch = mpatch.Rectangle(xy,P.w,P.h,fc='orange',ec='black')
			self.handles[0] = mass_patch
			self.ax.add_patch(mass_patch)
		else:
			self.handles[0].set_xy(xy)


	def drawSpring(self, z):
		n = 12
		l = z - P.w/2.0 - P.wall_pos
		h = P.h/2.0
		phi = np.arccos(l/(P.l))
		h2 = h + (l/n * np.tan(phi))
		xs = [P.wall_pos + l * x/n for x in xrange(n+1) ]
		ys = [h, h2] * (n/2)
		ys.append(h)

		if self.initflag:
			spring_path, = self.ax.plot(xs,ys,lw=2,c='black')
			self.handles[1] = spring_path
		else:
			self.handles[1].set_xdata(xs)
			self.handles[1].set_ydata(ys)


	def drawDamper(self, z):
		l = z - P.w/2.0 - P.wall_pos
		h = P.h/3.0
		w = 1.5 * (P.l)
		xs = (P.wall_pos, l)

		if self.initflag:
			damper, = self.ax.plot(xs,(h,h),lw=w,c='green')
			#damper, = plt.plot(xs,(h,h),lw=w,c='green')
			self.handles[2] = damper
		else:
			self.handles[2].set_xdata(xs)
			self.handles[2].set_linewidth(w)

	

if __name__ == '__main__':
	animator = MassSpringDamperAnimation()
	z = P.l/2.0 + P.w/2.0
	animator.draw(z)
	plt.show()
	
