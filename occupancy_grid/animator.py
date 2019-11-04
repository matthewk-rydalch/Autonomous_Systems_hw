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
		
		resized = cv2.resize(grid.T, size, interpolation = cv2.INTER_AREA)
		location = (int(xt[0]*scale), int(xt[1]*scale))
		cv2.circle(resized,location,2*scale,(255,255,0),-1)
		cv2.imshow('grid', resized)
		cv2.waitKey(1)