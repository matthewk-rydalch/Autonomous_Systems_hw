from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

class Animator:
    def animator(self, fig, ax, map, xt):
        
        ax.imshow(map, 'Greys')
        plt.show()
        # #initialize animations elements
        # robot, = plt.plot([], [], 'ro', markersize=12, animated=True)
        # grid, = plt.plot([], [], 'k*', markersize=3, animated=True)
        # particles, = plt.plot([], [], 'k.', markersize=3, animated=True)
        # f = np.linspace(-3, 3, 200)

        # def init():

        #     robot.set_data(xdata,ydata)
        #     arrow.set_data(xpoint, ypoint)
        #     particles.set_data(Xk[0][0,:], Xk[0][1,:])

        #     return robot, arrow, particles

        # def update(frame):

        #     #gather the right elements
        #     i = int(frame)
        #     xdata = self.xhat[i]
        #     ydata = self.yhat[i]
        #     thdata = self.thhat[i]
        #     xpoint = xdata + np.cos(thhat[i])
        #     ypoint = ydata + np.sin(thhat[i])

        #     #set the animation elements
        #     robot.set_data(xdata, ydata)
        #     arrow.set_data(xpoint, ypoint)
        #     particles.set_data(Xk[i][0,:], Xk[i][1,:])

        #     return robot, arrow, particles

        # #run the animation
        # ani = animation.FuncAnimation(fig, update,
        #                     init_func=init, frames = 201, interval = 20, blit=True)
        # plt.show()
