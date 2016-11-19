import numpy as np

class ControlTrace(object):

	def __init__(self, num_points, num_references=1, num_forces=1, 
		         num_outputs=1, num_observers=0):

		if num_references < 1:
			raise ValueError('There must be at least one reference')
		self.references = np.zeros((num_points, num_references))
		self.next_ref = 0

		if num_forces < 1:
			raise ValueError('There must be at least one force')
		self.forces = np.zeros((num_points, num_forces))
		self.next_force = 0

		if num_outputs < 1:
			raise ValueError('There must be at least one output')
		self.outputs = np.zeros((num_points, num_outputs))
		self.next_out = 0

		if num_observers > 0:
			self.observers = np.zeros((num_points, num_observers))
			self.next_obs = 0


	def update_all(self, ref, force, output, observer=None):

		self.references[self.next_ref] = ref
		self.next_ref += 1

		self.forces[self.next_force] = force
		self.next_force += 1
		
		self.outputs[self.next_out] = output
		self.next_out += 1

		if observer:
			self.observers[self.next_obs] = observer
			self.next_obs += 1