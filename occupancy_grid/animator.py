from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import cv2

class Animator:
	def animator(self, grid, xt):
		
		scale = 9
		x_size = 100*scale
		y_size = 100*scale
		size = (x_size, y_size)
		th = -np.pi/2
		rotate = np.array([[np.cos(th), -np.sin(th)],\
				          [np.sin(th), np.cos(th)]])
		grid_rotate = np.zeros((len(grid),len(grid)))
		for i in range(len(grid)):
			for j in range(len(grid)):
				rotate_ind = np.array([[i, j]])@rotate
				grid_rotate[int(rotate_ind[0][0])][int(rotate_ind[0][1])] = grid[i][j]
		
		resized = cv2.resize(grid_rotate, size, interpolation = cv2.INTER_AREA)
		# location = (int(xt[0]*scale), int(xt[1]*scale))
		location = (int(xt[0]), int(xt[1]))@rotate
		location = (int(location[0]*scale),int(location[1]*scale))
		cv2.circle(resized,location,2*scale,(255,255,0),-1)
		cv2.imshow('grid', resized)
		cv2.waitKey(1)