from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import cv2

class Animator:
	def animator(self, map, xt):
		
		x_size = 1000
		y_size = 1000
		size = (x_size, y_size)
		resized = cv2.resize(map, size, interpolation = cv2.INTER_AREA)
		location = [xt[0], xt[1]]
		# robot = np.array([location+[3,3],location+[-3,3],\
		# 	location+[-3,-3],location+[3,-3]])
		robot = np.array([50,50], dtype=np.int32)
		set_trace()
		cv2.polylines(np.uint8(resized), robot, isClosed=False, color=(255,0,0))
		# cv2.fillPoly(np.uint8(resized), np.int32(robot), 255)
		cv2.imshow('grid', resized)
		cv2.waitKey(10)


# a3 = np.array( [[[10,10],[100,10],[100,100],[10,100]]], dtype=np.int32 )
# im = np.zeros([240,320],dtype=np.uint8)
# cv2.fillPoly( im, a3, 255 )

# plt.imshow(im)
# plt.show()