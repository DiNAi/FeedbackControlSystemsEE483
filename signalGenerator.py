

def squareWave(min, max, hz, Ts):
	"""An infinite square wave that oscillates between min and max
	witth period 1 / hz. Ts is the timestep."""
	period = 1.0 / hz
	a = [min, max]
	i = 0
	while True:
		for _ in xrange(int(period / Ts / 2.0)):
			yield a[i]
		i = 1-i